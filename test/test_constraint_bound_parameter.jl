# Toy formulation for ConstraintBoundParameter: gen_a carries a "bound" time series b_t and
# the Interval row `p[gen_a, t] <= BOUND_TEST_RHS`, free where b_t is zero; gen_b is the
# expensive backup.
struct TestBoundParameter <: PSI.ConstraintBoundParameter end
struct TestBoundConstraint <: PSI.ConstraintType end
struct BoundTestDispatch <: PSI.AbstractThermalDispatchFormulation end

const BOUND_TEST_RHS = 0.8
const BOUND_TEST_COST = Dict("gen_a" => 10.0, "gen_b" => 50.0)
const BOUND_TEST_START = DateTime("2024-01-01T00:00:00")

PSI.get_multiplier_value(
    ::TestBoundParameter,
    ::PSY.ThermalStandard,
    ::BoundTestDispatch,
) = 1.0

function PSI.construct_device!(
    container::PSI.OptimizationContainer,
    sys::PSY.System,
    ::PSI.ArgumentConstructStage,
    model::PSI.DeviceModel{PSY.ThermalStandard, BoundTestDispatch},
    network_model::PSI.NetworkModel{<:PM.AbstractActivePowerModel},
)
    devices = PSI.get_available_components(model, sys)
    PSI.add_variables!(container, PSI.ActivePowerVariable, devices, BoundTestDispatch())
    PSI.add_parameters!(container, TestBoundParameter, devices, model)
    PSI.add_to_expression!(
        container,
        PSI.ActivePowerBalance,
        PSI.ActivePowerVariable,
        devices,
        model,
        network_model,
    )
    return
end

function PSI.construct_device!(
    container::PSI.OptimizationContainer,
    sys::PSY.System,
    ::PSI.ModelConstructStage,
    model::PSI.DeviceModel{PSY.ThermalStandard, BoundTestDispatch},
    network_model::PSI.NetworkModel{<:PM.AbstractActivePowerModel},
)
    devices = PSI.get_available_components(model, sys)
    time_steps = PSI.get_time_steps(container)
    jump_model = PSI.get_jump_model(container)
    p = PSI.get_variable(container, PSI.ActivePowerVariable(), PSY.ThermalStandard)
    key = PSI.ParameterKey(TestBoundParameter, PSY.ThermalStandard)
    names = collect(PSI.get_component_names(PSI.get_parameter_attributes(container, key)))
    cons = PSI.add_constraints_container!(
        container,
        TestBoundConstraint(),
        PSY.ThermalStandard,
        names,
        time_steps,
    )
    for name in names, t in time_steps
        cons[name, t] = JuMP.@constraint(
            jump_model,
            1.0 * p[name, t] in MOI.Interval(-Inf, BOUND_TEST_RHS)
        )
        PSI.add_bound_target!(container, key, name, t, cons[name, t], (-Inf, BOUND_TEST_RHS))
    end
    for d in devices, t in time_steps
        name = PSY.get_name(d)
        PSI.add_to_objective_invariant_expression!(
            container,
            BOUND_TEST_COST[name] * p[name, t],
        )
    end
    return
end

function _bound_test_system()
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
    for name in ("gen_a", "gen_b")
        gen = PSY.ThermalStandard(;
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
            operation_cost = PSY.ThermalGenerationCost(nothing),
            base_power = 100.0,
            prime_mover_type = PSY.PrimeMovers.ST,
            fuel = PSY.ThermalFuels.COAL,
        )
        PSY.add_component!(sys, gen)
    end
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
    timestamps = range(BOUND_TEST_START; step = Hour(1), length = 3)
    PSY.add_time_series!(
        sys,
        load,
        PSY.SingleTimeSeries("max_active_power", TimeArray(timestamps, [1.0, 1.0, 1.0])),
    )
    PSY.add_time_series!(
        sys,
        PSY.get_component(PSY.ThermalStandard, sys, "gen_a"),
        PSY.SingleTimeSeries("bound", TimeArray(timestamps, [1.0, 0.0, 1.0])),
    )
    PSY.transform_single_time_series!(sys, Hour(1), Hour(1))
    return sys
end

function _bound_test_template()
    template = ProblemTemplate(
        NetworkModel(CopperPlatePowerModel; duals = [CopperPlateBalanceConstraint]),
    )
    set_device_model!(
        template,
        DeviceModel(
            ThermalStandard,
            BoundTestDispatch;
            time_series_names = Dict(
                TestBoundParameter => "bound",
                FuelCostParameter => "fuel_cost",
            ),
        ),
    )
    set_device_model!(template, PowerLoad, StaticPowerLoad)
    return template
end

function _bound_test_set(container, name)
    cons = PSI.get_constraint(container, TestBoundConstraint(), ThermalStandard)
    return MOI.get(JuMP.owner_model(cons[name, 1]), MOI.ConstraintSet(), cons[name, 1])
end

@testset "ConstraintBoundParameter toggles a row between bounded and free across steps" begin
    output_dir = mktempdir(; cleanup = true)
    model = DecisionModel(
        _bound_test_template(),
        _bound_test_system();
        name = "ED",
        optimizer = HiGHS_optimizer,
    )
    models = SimulationModels(; decision_models = [model])
    sim = Simulation(;
        name = "bound_test",
        steps = 2,
        models = models,
        sequence = SimulationSequence(;
            models = models,
            ini_cond_chronology = InterProblemChronology(),
        ),
        initial_time = BOUND_TEST_START,
        simulation_folder = output_dir,
    )
    @test build!(sim) == PSI.SimulationBuildStatus.BUILT
    container = PSI.get_optimization_container(model)
    key = PSI.ParameterKey(TestBoundParameter, ThermalStandard)
    @test eltype(PSI.get_parameter_array(container, key)) == Float64
    # Step 1 is active at build.
    @test _bound_test_set(container, "gen_a") == MOI.Interval(-Inf, BOUND_TEST_RHS)
    @test execute!(sim) == PSI.RunStatus.SUCCESSFULLY_FINALIZED
    # After step 2 the row is free.
    @test _bound_test_set(container, "gen_a") == MOI.Interval(-Inf, Inf)

    res = get_decision_problem_results(SimulationResults(sim), "ED")
    power = read_realized_variable(
        res,
        ActivePowerVariable,
        ThermalStandard;
        table_format = TableFormat.WIDE,
    )
    bound = read_realized_parameter(
        res,
        TestBoundParameter,
        ThermalStandard;
        table_format = TableFormat.WIDE,
    )
    @test bound[!, "gen_a"] == [1.0, 0.0]
    # 0.8 pu cap while bounded and no cap while free, in MW.
    @test isapprox(power[!, "gen_a"], [80.0, 100.0]; atol = 1e-6)
    @test isapprox(power[!, "gen_b"], [20.0, 0.0]; atol = 1e-6)
end

@testset "ConstraintBoundParameter standalone build sets the bounds" begin
    for (offset, expected) in (
        (Hour(0), MOI.Interval(-Inf, BOUND_TEST_RHS)),
        (Hour(1), MOI.Interval(-Inf, Inf)),
    )
        model = DecisionModel(
            _bound_test_template(),
            _bound_test_system();
            name = "standalone",
            optimizer = HiGHS_optimizer,
            initial_time = BOUND_TEST_START + offset,
        )
        @test build!(model; output_dir = mktempdir(; cleanup = true)) ==
              PSI.ModelBuildStatus.BUILT
        @test _bound_test_set(PSI.get_optimization_container(model), "gen_a") == expected
    end
end

@testset "add_bound_target! rejects rebuild_model" begin
    model = DecisionModel(
        _bound_test_template(),
        _bound_test_system();
        name = "rebuild",
        optimizer = HiGHS_optimizer,
        rebuild_model = true,
    )
    @test build!(
        model;
        output_dir = mktempdir(; cleanup = true),
        console_level = Logging.AboveMaxLevel,
    ) == PSI.ModelBuildStatus.FAILED
end

@testset "numerical bounds ignore the infinite bounds of a free row" begin
    bounds = PSI.NumericalBounds()
    PSI.update_numerical_bounds(bounds, MOI.Interval(-Inf, Inf), ("row", 1))
    @test bounds.min == Inf && bounds.max == -Inf
    PSI.update_numerical_bounds(bounds, MOI.Interval(-Inf, 2.0), ("row", 2))
    @test bounds.min == 2.0 && bounds.max == -Inf
end

@testset "exporting a model with free and one-sided Interval rows" begin
    model = JuMP.Model()
    JuMP.@variable(model, x >= 0)
    JuMP.@constraint(model, 1.0 * x in MOI.Interval(-Inf, Inf))
    JuMP.@constraint(model, 2.0 * x in MOI.Interval(-Inf, 3.0))
    JuMP.@constraint(model, 3.0 * x in MOI.Interval(1.0, Inf))
    JuMP.@constraint(model, 4.0 * x in MOI.Interval(1.0, 5.0))
    path = joinpath(mktempdir(; cleanup = true), "model.mof.json")
    PSI.serialize_jump_optimization_model(model, path)
    @test isfile(path)
    # The live model keeps all four rows.
    @test JuMP.num_constraints(model, JuMP.AffExpr, MOI.Interval{Float64}) == 4
end
