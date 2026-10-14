# Toy formulations for ConstraintCoefficientParameter: gen_a carries a "coefficient" time
# series c_t and the row `c_t * p[gen_a, t] <= COEF_TEST_RHS`; gen_b is the expensive backup.
struct TestCoefficientParameter <: PSI.ConstraintCoefficientParameter end
struct TestCoefficientConstraint <: PSI.ConstraintType end
struct TestOnLinkConstraint <: PSI.ConstraintType end
struct CoefficientTestDispatch <: PSI.AbstractThermalDispatchFormulation end
struct CoefficientTestCommitment <: PSI.AbstractThermalDispatchFormulation end

const CoefficientTestFormulation = Union{CoefficientTestDispatch, CoefficientTestCommitment}
const COEF_TEST_RHS = 0.8
const COEF_TEST_COST = Dict("gen_a" => 10.0, "gen_b" => 50.0)
const COEF_TEST_ON_COST = 1.0
const COEF_TEST_START = DateTime("2024-01-01T00:00:00")

_coef_test_uses_on(::Type{CoefficientTestDispatch}) = false
_coef_test_uses_on(::Type{CoefficientTestCommitment}) = true

PSI.get_multiplier_value(
    ::TestCoefficientParameter,
    ::PSY.ThermalStandard,
    ::CoefficientTestFormulation,
) = 1.0

function PSI.construct_device!(
    container::PSI.OptimizationContainer,
    sys::PSY.System,
    ::PSI.ArgumentConstructStage,
    model::PSI.DeviceModel{PSY.ThermalStandard, F},
    network_model::PSI.NetworkModel{<:PM.AbstractActivePowerModel},
) where {F <: CoefficientTestFormulation}
    devices = PSI.get_available_components(model, sys)
    PSI.add_variables!(container, PSI.ActivePowerVariable, devices, F())
    if _coef_test_uses_on(F)
        PSI.add_variables!(container, PSI.OnVariable, devices, F())
    end
    PSI.add_parameters!(container, TestCoefficientParameter, devices, model)
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
    model::PSI.DeviceModel{PSY.ThermalStandard, F},
    network_model::PSI.NetworkModel{<:PM.AbstractActivePowerModel},
) where {F <: CoefficientTestFormulation}
    devices = PSI.get_available_components(model, sys)
    time_steps = PSI.get_time_steps(container)
    jump_model = PSI.get_jump_model(container)
    p = PSI.get_variable(container, PSI.ActivePowerVariable(), PSY.ThermalStandard)
    key = PSI.ParameterKey(TestCoefficientParameter, PSY.ThermalStandard)
    names = collect(PSI.get_component_names(PSI.get_parameter_attributes(container, key)))
    cons = PSI.add_constraints_container!(
        container,
        TestCoefficientConstraint(),
        PSY.ThermalStandard,
        names,
        time_steps,
    )
    for name in names, t in time_steps
        cons[name, t] = JuMP.@constraint(jump_model, 1.0 * p[name, t] <= COEF_TEST_RHS)
        PSI.add_coefficient_target!(container, key, name, t, cons[name, t], p[name, t], 1.0)
    end
    for d in devices, t in time_steps
        name = PSY.get_name(d)
        PSI.add_to_objective_invariant_expression!(
            container,
            COEF_TEST_COST[name] * p[name, t],
        )
    end
    if _coef_test_uses_on(F)
        on = PSI.get_variable(container, PSI.OnVariable(), PSY.ThermalStandard)
        all_names = PSY.get_name.(devices)
        link = PSI.add_constraints_container!(
            container,
            TestOnLinkConstraint(),
            PSY.ThermalStandard,
            all_names,
            time_steps,
        )
        for d in devices, t in time_steps
            name = PSY.get_name(d)
            p_max = PSY.get_active_power_limits(d).max
            link[name, t] =
                JuMP.@constraint(jump_model, p[name, t] - p_max * on[name, t] <= 0.0)
            PSI.add_to_objective_invariant_expression!(
                container,
                COEF_TEST_ON_COST * on[name, t],
            )
        end
    end
    return
end

function _coefficient_test_system()
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
    timestamps = range(COEF_TEST_START; step = Hour(1), length = 3)
    PSY.add_time_series!(
        sys,
        load,
        PSY.SingleTimeSeries("max_active_power", TimeArray(timestamps, [1.0, 1.0, 1.0])),
    )
    PSY.add_time_series!(
        sys,
        PSY.get_component(PSY.ThermalStandard, sys, "gen_a"),
        PSY.SingleTimeSeries("coefficient", TimeArray(timestamps, [1.0, 2.0, 4.0])),
    )
    PSY.transform_single_time_series!(sys, Hour(1), Hour(1))
    return sys
end

function _coefficient_test_template(formulation)
    template = ProblemTemplate(
        NetworkModel(CopperPlatePowerModel; duals = [CopperPlateBalanceConstraint]),
    )
    set_device_model!(
        template,
        DeviceModel(
            ThermalStandard,
            formulation;
            time_series_names = Dict(
                TestCoefficientParameter => "coefficient",
                FuelCostParameter => "fuel_cost",
            ),
        ),
    )
    set_device_model!(template, PowerLoad, StaticPowerLoad)
    return template
end

function _coefficient_test_standalone(formulation, initial_time, output_dir)
    model = DecisionModel(
        _coefficient_test_template(formulation),
        _coefficient_test_system();
        name = "standalone",
        optimizer = HiGHS_optimizer,
        initial_time = initial_time,
    )
    @test build!(model; output_dir = output_dir) == PSI.ModelBuildStatus.BUILT
    @test solve!(model) == PSI.RunStatus.SUCCESSFULLY_FINALIZED
    res = OptimizationProblemResults(model)
    power = read_variable(
        res,
        ActivePowerVariable,
        ThermalStandard;
        table_format = TableFormat.WIDE,
    )
    dual = read_dual(
        res,
        CopperPlateBalanceConstraint,
        PSY.System;
        table_format = TableFormat.WIDE,
    )
    return (
        objective = get_objective_value(res),
        gen_a = power[1, "gen_a"],
        gen_b = power[1, "gen_b"],
        dual = dual[1, "1"],
    )
end

function _coefficient_test_simulation(formulation, output_dir)
    model = DecisionModel(
        _coefficient_test_template(formulation),
        _coefficient_test_system();
        name = "ED",
        optimizer = HiGHS_optimizer,
    )
    sim = Simulation(;
        name = "coefficient_test",
        steps = 2,
        models = SimulationModels(; decision_models = [model]),
        sequence = SimulationSequence(;
            models = SimulationModels(; decision_models = [model]),
            ini_cond_chronology = InterProblemChronology(),
        ),
        initial_time = COEF_TEST_START,
        simulation_folder = output_dir,
    )
    @test build!(sim) == PSI.SimulationBuildStatus.BUILT
    @test execute!(sim) == PSI.RunStatus.SUCCESSFULLY_FINALIZED
    return sim, model
end

function _check_coefficient_simulation(formulation)
    output_dir = mktempdir(; cleanup = true)
    sim, model = _coefficient_test_simulation(formulation, output_dir)
    container = PSI.get_optimization_container(model)
    key = PSI.ParameterKey(TestCoefficientParameter, ThermalStandard)
    # Float64 storage in a recurrent build: the coefficient is not a JuMP parameter.
    @test eltype(PSI.get_parameter_array(container, key)) == Float64
    # After step 2 the row reads 2.0 * p[gen_a] <= 0.8.
    cons = PSI.get_constraint(container, TestCoefficientConstraint(), ThermalStandard)
    p = PSI.get_variable(container, ActivePowerVariable(), ThermalStandard)
    @test JuMP.normalized_coefficient(cons["gen_a", 1], p["gen_a", 1]) == 2.0
    # The MILP variant runs the dual pass (binaries fixed, LP re-solve, binaries restored).
    @test PSI.is_milp(container) == _coef_test_uses_on(formulation)

    results = SimulationResults(sim)
    res = get_decision_problem_results(results, "ED")
    power = read_realized_variable(
        res,
        ActivePowerVariable,
        ThermalStandard;
        table_format = TableFormat.WIDE,
    )
    coefficient = read_realized_parameter(
        res,
        TestCoefficientParameter,
        ThermalStandard;
        table_format = TableFormat.WIDE,
    )
    duals = read_realized_dual(
        res,
        CopperPlateBalanceConstraint,
        PSY.System;
        table_format = TableFormat.WIDE,
    )
    stats = read_optimizer_stats(res)
    @test coefficient[!, "gen_a"] == [1.0, 2.0]
    for (step, initial_time) in
        enumerate((COEF_TEST_START, COEF_TEST_START + Hour(1)))
        standalone = _coefficient_test_standalone(formulation, initial_time, output_dir)
        @test isapprox(stats[step, "objective_value"], standalone.objective; atol = 1e-6)
        @test isapprox(power[step, "gen_a"], standalone.gen_a; atol = 1e-6)
        @test isapprox(power[step, "gen_b"], standalone.gen_b; atol = 1e-6)
        @test isapprox(duals[step, "1"], standalone.dual; atol = 1e-6)
    end
    # 0.8 pu cap at c = 1 and 0.4 pu at c = 2, in MW.
    @test isapprox(power[!, "gen_a"], [80.0, 40.0]; atol = 1e-6)
    @test isapprox(power[!, "gen_b"], [20.0, 60.0]; atol = 1e-6)
    return
end

@testset "ConstraintCoefficientParameter LP simulation" begin
    _check_coefficient_simulation(CoefficientTestDispatch)
end

@testset "ConstraintCoefficientParameter MILP simulation with dual pass" begin
    _check_coefficient_simulation(CoefficientTestCommitment)
end

@testset "ConstraintCoefficientParameter standalone build sets the coefficient" begin
    output_dir = mktempdir(; cleanup = true)
    model = DecisionModel(
        _coefficient_test_template(CoefficientTestDispatch),
        _coefficient_test_system();
        name = "standalone",
        optimizer = HiGHS_optimizer,
        initial_time = COEF_TEST_START + Hour(1),
    )
    @test build!(model; output_dir = output_dir) == PSI.ModelBuildStatus.BUILT
    container = PSI.get_optimization_container(model)
    cons = PSI.get_constraint(container, TestCoefficientConstraint(), ThermalStandard)
    p = PSI.get_variable(container, ActivePowerVariable(), ThermalStandard)
    @test JuMP.normalized_coefficient(cons["gen_a", 1], p["gen_a", 1]) == 2.0
    @test axes(cons, 1) == ["gen_a"]
end

@testset "add_coefficient_target! rejects rebuild_model" begin
    output_dir = mktempdir(; cleanup = true)
    model = DecisionModel(
        _coefficient_test_template(CoefficientTestDispatch),
        _coefficient_test_system();
        name = "rebuild",
        optimizer = HiGHS_optimizer,
        rebuild_model = true,
    )
    @test build!(model; output_dir = output_dir, console_level = Logging.AboveMaxLevel) ==
          PSI.ModelBuildStatus.FAILED
end
