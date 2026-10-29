# Toy formulations for meta-keyed time-series parameters. Every thermal device carries
# the Deterministic series "series_a", "series_b" and "series_c". The default formulation
# registers one parameter per series under meta keys "a" and "b", a specialized
# parameter that is routed through a test `_add_parameters!` method, and an unkeyed
# parameter that reads the series named by the model's time_series_names.
struct TestMetaParameter <: PSI.TimeSeriesParameter end
struct TestMetaDefaultParameter <: PSI.TimeSeriesParameter end
struct TestMetaSpecializedParameter <: PSI.TimeSeriesParameter end
struct TestMetaSubsetParameter <: PSI.TimeSeriesParameter end
struct TestMetaCoefficientParameter <: PSI.ConstraintCoefficientParameter end
struct TestMetaCoefficientConstraint <: PSI.ConstraintType end
struct TestMetaDispatch <: PSI.AbstractThermalDispatchFormulation end
struct TestMetaRebuildDispatch <: PSI.AbstractThermalDispatchFormulation end
struct TestMetaSubsetDispatch <: PSI.AbstractThermalDispatchFormulation end
struct TestMetaCoefficientDispatch <: PSI.AbstractThermalDispatchFormulation end

const META_TEST_START = DateTime("2024-01-01T00:00:00")
const META_TEST_DEVICES = ("gen_a", "gen_b")
const META_COEF_RHS = 0.8
const META_SPECIALIZED_CALLS = Tuple{String, Union{Nothing, String}}[]

PSI.get_multiplier_value(
    ::Union{TestMetaParameter, TestMetaDefaultParameter, TestMetaSpecializedParameter},
    ::PSY.ThermalStandard,
    ::TestMetaDispatch,
) = 1.0
PSI.get_multiplier_value(
    ::TestMetaParameter,
    ::PSY.ThermalStandard,
    ::TestMetaRebuildDispatch,
) = 1.0
PSI.get_multiplier_value(
    ::TestMetaSubsetParameter,
    ::PSY.ThermalStandard,
    ::TestMetaSubsetDispatch,
) = 1.0
PSI.get_multiplier_value(
    ::TestMetaCoefficientParameter,
    ::PSY.ThermalStandard,
    ::TestMetaCoefficientDispatch,
) = 1.0

# Downstream-style specialization: records the call and creates no container.
function PSI._add_parameters!(
    ::PSI.OptimizationContainer,
    ::TestMetaSpecializedParameter,
    devices::U,
    ::PSI.DeviceModel{D, W};
    meta::String,
    time_series_name::Union{Nothing, String},
) where {
    U <: Union{Vector{D}, IS.FlattenIteratorWrapper{D}},
    W <: PSI.AbstractDeviceFormulation,
} where {D <: PSY.Component}
    push!(META_SPECIALIZED_CALLS, (meta, time_series_name))
    return
end

function PSI.construct_device!(
    container::PSI.OptimizationContainer,
    sys::PSY.System,
    ::PSI.ArgumentConstructStage,
    model::PSI.DeviceModel{PSY.ThermalStandard, TestMetaDispatch},
    network_model::PSI.NetworkModel{<:PM.AbstractActivePowerModel},
)
    devices = PSI.get_available_components(model, sys)
    PSI.add_variables!(container, PSI.ActivePowerVariable, devices, TestMetaDispatch())
    PSI.add_parameters!(
        container,
        TestMetaParameter,
        devices,
        model;
        meta = "a",
        time_series_name = "series_a",
    )
    PSI.add_parameters!(
        container,
        TestMetaParameter,
        devices,
        model;
        meta = "b",
        time_series_name = "series_b",
    )
    PSI.add_parameters!(container, TestMetaDefaultParameter, devices, model)
    PSI.add_parameters!(
        container,
        TestMetaSpecializedParameter,
        devices,
        model;
        meta = "s",
        time_series_name = "series_a",
    )
    return
end

function PSI.construct_device!(
    ::PSI.OptimizationContainer,
    ::PSY.System,
    ::PSI.ModelConstructStage,
    ::PSI.DeviceModel{PSY.ThermalStandard, TestMetaDispatch},
    ::PSI.NetworkModel{<:PM.AbstractActivePowerModel},
)
    return
end

function PSI.construct_device!(
    container::PSI.OptimizationContainer,
    sys::PSY.System,
    ::PSI.ArgumentConstructStage,
    model::PSI.DeviceModel{PSY.ThermalStandard, TestMetaRebuildDispatch},
    network_model::PSI.NetworkModel{<:PM.AbstractActivePowerModel},
)
    devices = PSI.get_available_components(model, sys)
    PSI.add_variables!(
        container,
        PSI.ActivePowerVariable,
        devices,
        TestMetaRebuildDispatch(),
    )
    PSI.add_parameters!(
        container,
        TestMetaParameter,
        devices,
        model;
        meta = "a",
        time_series_name = "series_a",
    )
    # Second add with the same meta: the rebuild guard must skip it.
    PSI.add_parameters!(
        container,
        TestMetaParameter,
        devices,
        model;
        meta = "a",
        time_series_name = "series_b",
    )
    return
end

function PSI.construct_device!(
    ::PSI.OptimizationContainer,
    ::PSY.System,
    ::PSI.ModelConstructStage,
    ::PSI.DeviceModel{PSY.ThermalStandard, TestMetaRebuildDispatch},
    ::PSI.NetworkModel{<:PM.AbstractActivePowerModel},
)
    return
end

function PSI.construct_device!(
    container::PSI.OptimizationContainer,
    sys::PSY.System,
    ::PSI.ArgumentConstructStage,
    model::PSI.DeviceModel{PSY.ThermalStandard, TestMetaSubsetDispatch},
    network_model::PSI.NetworkModel{<:PM.AbstractActivePowerModel},
)
    devices = PSI.get_available_components(model, sys)
    PSI.add_variables!(
        container,
        PSI.ActivePowerVariable,
        devices,
        TestMetaSubsetDispatch(),
    )
    # Only gen_a is registered, although gen_b carries the same series.
    subset = [d for d in devices if PSY.get_name(d) == "gen_a"]
    PSI.add_parameters!(
        container,
        TestMetaSubsetParameter,
        subset,
        model;
        meta = "c",
        time_series_name = "series_c",
    )
    return
end

function PSI.construct_device!(
    ::PSI.OptimizationContainer,
    ::PSY.System,
    ::PSI.ModelConstructStage,
    ::PSI.DeviceModel{PSY.ThermalStandard, TestMetaSubsetDispatch},
    ::PSI.NetworkModel{<:PM.AbstractActivePowerModel},
)
    return
end

function PSI.construct_device!(
    container::PSI.OptimizationContainer,
    sys::PSY.System,
    ::PSI.ArgumentConstructStage,
    model::PSI.DeviceModel{PSY.ThermalStandard, TestMetaCoefficientDispatch},
    network_model::PSI.NetworkModel{<:PM.AbstractActivePowerModel},
)
    devices = PSI.get_available_components(model, sys)
    PSI.add_variables!(
        container,
        PSI.ActivePowerVariable,
        devices,
        TestMetaCoefficientDispatch(),
    )
    PSI.add_parameters!(
        container,
        TestMetaCoefficientParameter,
        devices,
        model;
        meta = "k",
        time_series_name = "series_a",
    )
    return
end

function PSI.construct_device!(
    container::PSI.OptimizationContainer,
    sys::PSY.System,
    ::PSI.ModelConstructStage,
    model::PSI.DeviceModel{PSY.ThermalStandard, TestMetaCoefficientDispatch},
    network_model::PSI.NetworkModel{<:PM.AbstractActivePowerModel},
)
    time_steps = PSI.get_time_steps(container)
    jump_model = PSI.get_jump_model(container)
    p = PSI.get_variable(container, PSI.ActivePowerVariable(), PSY.ThermalStandard)
    key = PSI.ParameterKey(TestMetaCoefficientParameter, PSY.ThermalStandard, "k")
    names = collect(PSI.get_component_names(PSI.get_parameter_attributes(container, key)))
    cons = PSI.add_constraints_container!(
        container,
        TestMetaCoefficientConstraint(),
        PSY.ThermalStandard,
        names,
        time_steps,
    )
    for name in names, t in time_steps
        cons[name, t] = JuMP.@constraint(jump_model, 1.0 * p[name, t] <= META_COEF_RHS)
        PSI.add_coefficient_target!(
            container,
            TestMetaCoefficientParameter(),
            PSY.ThermalStandard,
            name,
            t,
            cons[name, t],
            p[name, t],
            1.0;
            meta = "k",
        )
    end
    return
end

# Parameter arrays are indexed by time-series UUID on their first axis. Simulation
# builds hold variables fixed to the current values, which are read back with `JuMP.fix_value`.
function _meta_test_value(container, key, name::String)
    attributes = PSI.get_parameter_attributes(container, key)
    array = PSI.get_parameter_array(container, key)
    value = array[PSI._get_ts_uuid(attributes, name), 1]
    return value isa JuMP.VariableRef ? JuMP.fix_value(value) : value
end

function _meta_test_system()
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
    for name in META_TEST_DEVICES
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
    timestamps = range(META_TEST_START; step = Hour(1), length = 3)
    for gen in PSY.get_components(PSY.ThermalStandard, sys)
        PSY.add_time_series!(
            sys,
            gen,
            PSY.SingleTimeSeries("series_a", TimeArray(timestamps, [1.0, 2.0, 4.0])),
        )
        PSY.add_time_series!(
            sys,
            gen,
            PSY.SingleTimeSeries("series_b", TimeArray(timestamps, [10.0, 20.0, 40.0])),
        )
        PSY.add_time_series!(
            sys,
            gen,
            PSY.SingleTimeSeries("series_c", TimeArray(timestamps, [100.0, 200.0, 400.0])),
        )
    end
    PSY.transform_single_time_series!(sys, Hour(1), Hour(1))
    return sys
end

function _meta_test_template(formulation)
    template = ProblemTemplate(NetworkModel(CopperPlatePowerModel))
    set_device_model!(
        template,
        DeviceModel(
            ThermalStandard,
            formulation;
            time_series_names = Dict(TestMetaDefaultParameter => "series_a"),
        ),
    )
    return template
end

function _meta_test_standalone(
    initial_time,
    output_dir;
    formulation = TestMetaDispatch,
    rebuild_model = false,
)
    model = DecisionModel(
        _meta_test_template(formulation),
        _meta_test_system();
        name = "standalone",
        optimizer = HiGHS_optimizer,
        initial_time = initial_time,
        rebuild_model = rebuild_model,
    )
    @test build!(model; output_dir = output_dir) == PSI.ModelBuildStatus.BUILT
    return model
end

function _meta_test_simulation(output_dir; formulation = TestMetaDispatch)
    model = DecisionModel(
        _meta_test_template(formulation),
        _meta_test_system();
        name = "ED",
        optimizer = HiGHS_optimizer,
    )
    sim = Simulation(;
        name = "meta_parameter_test",
        steps = 2,
        models = SimulationModels(; decision_models = [model]),
        sequence = SimulationSequence(;
            models = SimulationModels(; decision_models = [model]),
            ini_cond_chronology = InterProblemChronology(),
        ),
        initial_time = META_TEST_START,
        simulation_folder = output_dir,
    )
    @test build!(sim) == PSI.SimulationBuildStatus.BUILT
    @test execute!(sim) == PSI.RunStatus.SUCCESSFULLY_FINALIZED
    return sim, model
end

@testset "Meta-keyed time-series parameters build with explicit series" begin
    output_dir = mktempdir(; cleanup = true)
    model = _meta_test_standalone(META_TEST_START, output_dir)
    container = PSI.get_optimization_container(model)
    key_a = PSI.ParameterKey(TestMetaParameter, ThermalStandard, "a")
    key_b = PSI.ParameterKey(TestMetaParameter, ThermalStandard, "b")
    @test PSI.has_container_key(container, TestMetaParameter, ThermalStandard, "a")
    @test PSI.has_container_key(container, TestMetaParameter, ThermalStandard, "b")
    @test !PSI.has_container_key(container, TestMetaParameter, ThermalStandard)
    @test PSI.get_time_series_name(PSI.get_parameter_attributes(container, key_a)) ==
          "series_a"
    @test PSI.get_time_series_name(PSI.get_parameter_attributes(container, key_b)) ==
          "series_b"
    for name in META_TEST_DEVICES
        @test _meta_test_value(container, key_a, name) == 1.0
        @test _meta_test_value(container, key_b, name) == 10.0
    end
end

@testset "Unkeyed time-series parameters keep the model's series name" begin
    output_dir = mktempdir(; cleanup = true)
    model = _meta_test_standalone(META_TEST_START, output_dir)
    container = PSI.get_optimization_container(model)
    key = PSI.ParameterKey(TestMetaDefaultParameter, ThermalStandard)
    @test PSI.has_container_key(container, TestMetaDefaultParameter, ThermalStandard)
    @test PSI.get_time_series_name(PSI.get_parameter_attributes(container, key)) ==
          "series_a"
    for name in META_TEST_DEVICES
        @test _meta_test_value(container, key, name) == 1.0
    end
end

@testset "Meta-keyed time-series parameters update in a simulation" begin
    output_dir = mktempdir(; cleanup = true)
    _, model = _meta_test_simulation(output_dir)
    container = PSI.get_optimization_container(model)
    key_a = PSI.ParameterKey(TestMetaParameter, ThermalStandard, "a")
    key_b = PSI.ParameterKey(TestMetaParameter, ThermalStandard, "b")
    key_default = PSI.ParameterKey(TestMetaDefaultParameter, ThermalStandard)
    # The container holds the values of step 2, the second forecast window.
    for name in META_TEST_DEVICES
        @test _meta_test_value(container, key_a, name) == 2.0
        @test _meta_test_value(container, key_b, name) == 20.0
        @test _meta_test_value(container, key_default, name) == 2.0
    end
end

@testset "Meta-keyed parameters are read through the public results API" begin
    output_dir = mktempdir(; cleanup = true)
    sim, _ = _meta_test_simulation(output_dir)
    res = get_decision_problem_results(SimulationResults(sim), "ED")
    read(name) = read_realized_parameter(
        res,
        name;
        table_format = TableFormat.WIDE,
    )
    values_a = read("TestMetaParameter__ThermalStandard__a")
    values_b = read("TestMetaParameter__ThermalStandard__b")
    values_default = read("TestMetaDefaultParameter__ThermalStandard")
    for name in META_TEST_DEVICES
        @test values_a[!, name] == [1.0, 2.0]
        @test values_b[!, name] == [10.0, 20.0]
        @test values_default[!, name] == [1.0, 2.0]
    end
end

@testset "Specialized _add_parameters! is reached through the meta keyword path" begin
    empty!(META_SPECIALIZED_CALLS)
    output_dir = mktempdir(; cleanup = true)
    _meta_test_standalone(META_TEST_START, output_dir)
    @test META_SPECIALIZED_CALLS == [("s", "series_a")]
end

@testset "Meta-keyed parameter for a subset of devices updates in a simulation" begin
    output_dir = mktempdir(; cleanup = true)
    sim, _ = _meta_test_simulation(output_dir; formulation = TestMetaSubsetDispatch)
    res = get_decision_problem_results(SimulationResults(sim), "ED")
    values_c = read_realized_parameter(
        res,
        "TestMetaSubsetParameter__ThermalStandard__c";
        table_format = TableFormat.WIDE,
    )
    @test values_c[!, "gen_a"] == [100.0, 200.0]
    @test !("gen_b" in names(values_c))
end

@testset "Rebuild guard skips a second add with the same meta" begin
    output_dir = mktempdir(; cleanup = true)
    model = _meta_test_standalone(
        META_TEST_START,
        output_dir;
        formulation = TestMetaRebuildDispatch,
        rebuild_model = true,
    )
    container = PSI.get_optimization_container(model)
    key_a = PSI.ParameterKey(TestMetaParameter, ThermalStandard, "a")
    @test PSI.get_time_series_name(PSI.get_parameter_attributes(container, key_a)) ==
          "series_a"
    for name in META_TEST_DEVICES
        @test _meta_test_value(container, key_a, name) == 1.0
    end
end

@testset "ConstraintCoefficientParameter with meta and explicit series updates" begin
    output_dir = mktempdir(; cleanup = true)
    sim, model =
        _meta_test_simulation(output_dir; formulation = TestMetaCoefficientDispatch)
    container = PSI.get_optimization_container(model)
    cons = PSI.get_constraint(container, TestMetaCoefficientConstraint(), ThermalStandard)
    p = PSI.get_variable(container, ActivePowerVariable(), ThermalStandard)
    # The coefficient at step 2 is the series_a value 2.0.
    @test JuMP.normalized_coefficient(cons["gen_a", 1], p["gen_a", 1]) == 2.0
    res = get_decision_problem_results(SimulationResults(sim), "ED")
    coefficient = read_realized_parameter(
        res,
        "TestMetaCoefficientParameter__ThermalStandard__k";
        table_format = TableFormat.WIDE,
    )
    @test coefficient[!, "gen_a"] == [1.0, 2.0]
end
