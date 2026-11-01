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
    param = PSI.get_parameter(container, TestCoefficientParameter(), ThermalStandard)
    @test isempty(PSI.get_coefficient_targets(PSI.get_attributes(param)))
    @test !PSI.has_container_key(container, TestCoefficientConstraint, ThermalStandard)
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
    PSI.add_constraints!(
        container,
        TestCoefficientConstraint,
        devices,
        model,
        network_model,
    )
    time_steps = PSI.get_time_steps(container)
    jump_model = PSI.get_jump_model(container)
    p = PSI.get_variable(container, PSI.ActivePowerVariable(), PSY.ThermalStandard)
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

function PSI.add_constraints!(
    container::PSI.OptimizationContainer,
    ::Type{TestCoefficientConstraint},
    devices,
    model::PSI.DeviceModel{PSY.ThermalStandard, F},
    network_model::PSI.NetworkModel,
) where {F <: CoefficientTestFormulation}
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
    end
    selected = [d for d in devices if PSY.get_name(d) in names]
    PSI.add_constraint_coefficients!(
        container, TestCoefficientConstraint, PSI.ActivePowerVariable,
        TestCoefficientParameter, selected, model,
    )
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

struct OtherCoefficientParameter <: PSI.ConstraintCoefficientParameter end

function _binding_fixture(; names = ["a", "b"], constraint_names = names,
    variable_names = names, parameter_names = names, multiplier_names = names,
    constraint_meta = "constraints", variable_meta = "variables",
    parameter_meta = "parameters",
    component_type = ThermalStandard, rebuild_model = false)
    sys = PSY.System(100.0)
    container = PSI.OptimizationContainer(sys, PSI.Settings(sys; rebuild_model), nothing,
        PSY.Deterministic)
    container.time_steps = 1:2
    jump = PSI.get_jump_model(container)
    variables =
        PSI.add_variable_container!(container, ActivePowerVariable(), component_type,
            variable_names, 1:2; meta = variable_meta)
    for name in variable_names, t in 1:2
        variables[name, t] = @variable(jump)
    end
    constraints = PSI.add_constraints_container!(container, TestCoefficientConstraint(),
        component_type, constraint_names, 1:2; meta = constraint_meta)
    other = @variable(jump)
    for name in constraint_names, t in 1:2
        constraints[name, t] =
            @constraint(jump, 7.0 * variables[name, t] + 3.0 * other <= 10.0)
    end
    key = PSI.ParameterKey(TestCoefficientParameter, component_type, parameter_meta)
    attr = PSI.TimeSeriesAttributes(PSY.Deterministic, "coefficient")
    for name in parameter_names
        PSI.add_component_name!(attr, name, "shared-uuid")
    end
    param = PSI.ParameterContainer(attr,
        DenseAxisArray(fill(2.0, 1, 2), ["shared-uuid"], 1:2),
        DenseAxisArray(fill(3.0, length(multiplier_names), 2), multiplier_names, 1:2))
    container.parameters[key] = param
    return (; container, variables, constraints, other, key, param, attr)
end

function _bind_fixture(f; names = ["a", "b"], scale = -2.0, kwargs...)
    devices = [PSY.ThermalStandard(nothing) for _ in names]
    for (d, name) in zip(devices, names)
        PSY.set_name!(d, name)
    end
    model = DeviceModel(ThermalStandard, CoefficientTestDispatch;
        time_series_names = Dict(TestCoefficientParameter => "coefficient"))
    return PSI.add_constraint_coefficients!(f.container, TestCoefficientConstraint,
        ActivePowerVariable, TestCoefficientParameter, devices, model;
        constraint_meta = "constraints", variable_meta = "variables",
        parameter_meta = "parameters", scale, kwargs...)
end

@testset "Coefficient binding construction and replacement" begin
    f = _binding_fixture()
    f.param.multiplier_array["b", :] .= 4.0
    @test isempty(PSI.get_coefficient_targets(f.attr))
    @test _bind_fixture(f) === nothing
    @test length(PSI.get_coefficient_targets(f.attr)) == 4
    for t in 1:2
        @test normalized_coefficient(f.constraints["a", t], f.variables["a", t]) == -12.0
        @test normalized_coefficient(f.constraints["b", t], f.variables["b", t]) == -16.0
        @test normalized_coefficient(f.constraints["a", t], f.other) == 3.0
    end
    for value in [0.0, 5.0, 0.0, 1.0]
        f.param.parameter_array.data .= value
        PSI._apply_coefficient_targets!(f.param)
        @test normalized_coefficient(f.constraints["a", 1], f.variables["a", 1]) == -6value
        @test normalized_coefficient(f.constraints["b", 1], f.variables["b", 1]) == -8value
        @test normalized_coefficient(f.constraints["b", 1], f.other) == 3.0
    end
    other_key = PSI.ParameterKey(OtherCoefficientParameter, ThermalStandard)
    other_attr = PSI.TimeSeriesAttributes(PSY.Deterministic, "coefficient",
        1, Dict("a" => "shared-uuid"))
    f.container.parameters[other_key] = PSI.ParameterContainer(other_attr,
        f.param.parameter_array, f.param.multiplier_array)
    @test_throws r"already bound" PSI.add_coefficient_target!(f.container, other_key,
        "a", 1, f.constraints["a", 1], f.variables["a", 1], 1.0)
    @test isempty(PSI.get_coefficient_targets(other_attr))
    @test_throws r"already bound" _bind_fixture(f)
end

@testset "Coefficient binding prevalidation" begin
    for f in (_binding_fixture(; parameter_names = ["a"]),
        _binding_fixture(; multiplier_names = ["a"]),
        _binding_fixture(; constraint_names = ["a"]),
        _binding_fixture(; rebuild_model = true))
        @test_throws Exception _bind_fixture(f)
        @test isempty(PSI.get_coefficient_targets(f.attr))
        @test normalized_coefficient(f.constraints["a", 1], f.variables["a", 1]) == 7.0
    end
    f = _binding_fixture()
    for scale in (Inf, NaN)
        @test_throws r"finite" _bind_fixture(f; scale)
        @test isempty(PSI.get_coefficient_targets(f.attr))
    end
    f.param.multiplier_array["b", 2] = floatmax(Float64)
    @test_throws r"finite" _bind_fixture(f)
    @test isempty(PSI.get_coefficient_targets(f.attr))
    @test normalized_coefficient(f.constraints["a", 1], f.variables["a", 1]) == 7.0
    f.param.multiplier_array["b", 2] = 3.0
    @test_throws r"already bound" _bind_fixture(f; names = ["a", "a"])
    @test isempty(PSI.get_coefficient_targets(f.attr))
    foreign = JuMP.Model()
    x = @variable(foreign)
    con = @constraint(foreign, x <= 1)
    @test_throws r"model" PSI.add_coefficient_target!(f.container, f.key,
        "a", 1, con, x, 1.0)
    nonlinear = @constraint(PSI.get_jump_model(f.container), f.other^2 <= 1)
    @test_throws r"linear" PSI.add_coefficient_target!(f.container, f.key,
        "a", 1, nonlinear, f.other, 1.0)
    @test isempty(PSI.get_coefficient_targets(f.attr))
    _bind_fixture(f)
    f.param.parameter_array.data .= 10.0
    f.param.multiplier_array["b", 2] = Inf
    @test_throws r"finite" PSI._apply_coefficient_targets!(f.param)
    @test normalized_coefficient(f.constraints["a", 1], f.variables["a", 1]) == -12.0
end

@testset "Coefficient service binding" begin
    S = PSY.VariableReserve{PSY.ReserveUp}
    f = _binding_fixture(; names = ["reserve"], component_type = S,
        parameter_meta = "reserve")
    service = S(nothing)
    PSY.set_name!(service, "reserve")
    model = ServiceModel(S, RangeReserve, "reserve";
        time_series_names = Dict(TestCoefficientParameter => "coefficient"))
    @test PSI.add_constraint_coefficients!(f.container, TestCoefficientConstraint,
        ActivePowerVariable, TestCoefficientParameter, service, model;
        constraint_meta = "constraints", variable_meta = "variables", scale = -0.5) === nothing
    @test normalized_coefficient(f.constraints["reserve", 1], f.variables["reserve", 1]) ==
          -3.0
    @test length(PSI.get_coefficient_targets(f.attr)) == 2
    f.param.parameter_array.data .= 4.0
    PSI._apply_coefficient_targets!(f.param)
    @test normalized_coefficient(f.constraints["reserve", 2], f.variables["reserve", 2]) ==
          -6.0
end

@testset "Coefficient binding cell and model validation" begin
    f = _binding_fixture()
    jump = PSI.get_jump_model(f.container)
    for (name, t) in [("missing", 1), ("a", 3)]
        @test_throws Exception PSI.add_coefficient_target!(f.container, f.key, name, t,
            f.constraints["a", 1], f.variables["a", 1], 1.0)
    end
    bad_variable = @variable(JuMP.Model())
    @test_throws r"model" PSI.add_coefficient_target!(f.container, f.key, "a", 1,
        f.constraints["a", 1], bad_variable, 1.0)
    deleted = @constraint(jump, f.other <= 1)
    JuMP.delete(jump, deleted)
    @test_throws r"valid" PSI.add_coefficient_target!(f.container, f.key, "a", 1,
        deleted, f.other, 1.0)
    vkey = PSI.VariableKey(ActivePowerVariable, ThermalStandard, "variables")
    f.container.variables[vkey] = DenseAxisArray(f.variables.data[1:1, :], ["a"], 1:2)
    @test_throws r"variable" _bind_fixture(f)
    @test isempty(PSI.get_coefficient_targets(f.attr))
    f.container.variables[vkey] = f.variables
    f.container.parameters[f.key] = PSI.ParameterContainer(f.attr,
        DenseAxisArray(ones(1, 2, 1), ["shared-uuid"], 1:2, [1]),
        f.param.multiplier_array)
    @test_throws r"two-dimensional" _bind_fixture(f)
    @test isempty(PSI.get_coefficient_targets(f.attr))
    f.container.parameters[f.key] = PSI.ParameterContainer(f.attr,
        f.param.parameter_array,
        DenseAxisArray(f.param.multiplier_array.data, ["a", "b"], [2, 1]))
    @test_throws r"time" _bind_fixture(f)
    @test isempty(PSI.get_coefficient_targets(f.attr))
end

@testset "Zero initial binding and irregular explicit target" begin
    f = _binding_fixture()
    _bind_fixture(f; scale = 0.0)
    @test normalized_coefficient(f.constraints["a", 1], f.variables["a", 1]) == 0.0
    @test PSI.add_coefficient_target!(f.container, f.key, "a", 1,
        f.constraints["a", 1], f.variables["b", 2], -0.5) === nothing
    @test normalized_coefficient(f.constraints["a", 1], f.variables["b", 2]) == -3.0
    @test normalized_coefficient(f.constraints["a", 1], f.other) == 3.0
    f.param.parameter_array.data .= floatmax(Float64)
    f.param.multiplier_array["a", :] .= 4.0
    @test_throws r"finite" PSI._apply_coefficient_targets!(f.param)
    @test normalized_coefficient(f.constraints["a", 1], f.variables["b", 2]) == -3.0
end

@testset "Service binding rejects incompatible device axes" begin
    S = PSY.VariableReserve{PSY.ReserveUp}
    f = _binding_fixture(; names = ["reserve"], component_type = S,
        parameter_meta = "reserve")
    service = S(nothing)
    PSY.set_name!(service, "reserve")
    model = ServiceModel(S, RangeReserve, "reserve";
        time_series_names = Dict(TestCoefficientParameter => "coefficient"))
    f.container.variables[PSI.VariableKey(ActivePowerVariable, S, "variables")] =
        DenseAxisArray(f.variables.data, ["contributing-device"], 1:2)
    @test_throws r"variable.*reserve" PSI.add_constraint_coefficients!(f.container,
        TestCoefficientConstraint, ActivePowerVariable, TestCoefficientParameter,
        service, model; constraint_meta = "constraints", variable_meta = "variables")
    @test isempty(PSI.get_coefficient_targets(f.attr))
    @test_throws r"service-name metadata" PSI.add_constraint_coefficients!(f.container,
        TestCoefficientConstraint, ActivePowerVariable, TestCoefficientParameter,
        service, model; parameter_meta = "wrong-service")
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
