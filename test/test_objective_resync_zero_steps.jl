# A native time-varying cost parameter (fuel price) that is nonzero, then all zero, then
# nonzero must keep updating. A step whose variant cost contributions are all zero used to
# leave the objective marked synchronized, so the next native update raised an error.
const ZERO_STEP_START = DateTime("2024-01-01T00:00:00")
const ZERO_STEP_BACKUP_FUEL = 50.0

function _zero_step_system(prices::Vector{Float64})
    sys = PSY.System(100.0)
    bus = PSY.ACBus(;
        number = 1,
        name = "bus1",
        available = true,
        bustype = PSY.ACBusTypes.REF,
        angle = 0.0,
        magnitude = 1.0,
        voltage_limits = (min = 0.9, max = 1.1),
        base_voltage = 230.0,
    )
    PSY.add_component!(sys, bus)
    function _gen(name, fuel_cost)
        return PSY.ThermalStandard(;
            name = name,
            available = true,
            status = true,
            bus = bus,
            active_power = 0.0,
            reactive_power = 0.0,
            rating = 1.0,
            active_power_limits = (min = 0.0, max = 1.0),
            reactive_power_limits = nothing,
            ramp_limits = nothing,
            operation_cost = PSY.ThermalGenerationCost(;
                variable = PSY.FuelCurve(;
                    value_curve = PSY.LinearCurve(1.0),
                    fuel_cost = fuel_cost,
                ),
                fixed = 0.0,
                start_up = 0.0,
                shut_down = 0.0,
            ),
            base_power = 100.0,
            prime_mover_type = PSY.PrimeMovers.ST,
            fuel = PSY.ThermalFuels.COAL,
        )
    end
    gen_a = _gen("gen_a", 1.0)
    PSY.add_component!(sys, gen_a)
    PSY.add_component!(sys, _gen("gen_b", ZERO_STEP_BACKUP_FUEL))
    load = PSY.PowerLoad(;
        name = "load",
        available = true,
        bus = bus,
        active_power = 1.0,
        reactive_power = 0.0,
        base_power = 100.0,
        max_active_power = 1.0,
        max_reactive_power = 0.0,
    )
    PSY.add_component!(sys, load)
    n = length(prices)
    timestamps = range(ZERO_STEP_START; step = Hour(1), length = n)
    PSY.add_time_series!(
        sys,
        load,
        PSY.SingleTimeSeries("max_active_power", TimeArray(timestamps, ones(n))),
    )
    PSY.set_fuel_cost!(
        sys,
        gen_a,
        PSY.SingleTimeSeries("fuel_cost", TimeArray(timestamps, prices)),
    )
    PSY.transform_single_time_series!(sys, Hour(1), Hour(1))
    return sys
end

function _zero_step_template()
    template = ProblemTemplate(
        NetworkModel(CopperPlatePowerModel; duals = [CopperPlateBalanceConstraint]),
    )
    set_device_model!(template, ThermalStandard, ThermalBasicDispatch)
    set_device_model!(template, PowerLoad, StaticPowerLoad)
    return template
end

@testset "Native fuel cost parameter survives an all-zero step" begin
    prices = [1.0, 0.0, 3.0]
    output_dir = mktempdir(; cleanup = true)
    model = DecisionModel(
        _zero_step_template(),
        _zero_step_system(prices);
        name = "ED",
        optimizer = HiGHS_optimizer,
        initial_time = ZERO_STEP_START,
    )
    sim = Simulation(;
        name = "zero_step",
        steps = length(prices),
        models = SimulationModels(; decision_models = [model]),
        sequence = SimulationSequence(;
            models = SimulationModels(; decision_models = [model]),
            ini_cond_chronology = InterProblemChronology(),
        ),
        initial_time = ZERO_STEP_START,
        simulation_folder = output_dir,
    )
    @test build!(sim) == PSI.SimulationBuildStatus.BUILT
    @test execute!(sim) == PSI.RunStatus.SUCCESSFULLY_FINALIZED

    res = get_decision_problem_results(SimulationResults(sim), "ED")
    stats = read_optimizer_stats(res)
    power = read_realized_variable(
        res,
        ActivePowerVariable,
        ThermalStandard;
        table_format = TableFormat.WIDE,
    )
    # gen_a is the only cheap unit, so it carries the load at every step.
    @test all(isapprox.(power[!, "gen_a"], 100.0; atol = 1e-6))
    @test all(isapprox.(power[!, "gen_b"], 0.0; atol = 1e-6))
    # Step 1 fixes the cost per unit of fuel price; zero fuel costs nothing; step 3 scales it.
    unit_cost = stats[1, "objective_value"]
    @test unit_cost > 0.0
    @test isapprox(stats[2, "objective_value"], 0.0; atol = 1e-6)
    @test isapprox(stats[3, "objective_value"], 3.0 * unit_cost; atol = 1e-6)
end
