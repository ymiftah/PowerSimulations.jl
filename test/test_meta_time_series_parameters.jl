# Toy formulation for meta-keyed time-series parameters: gen_a and gen_b each carry
# two Deterministic series, "series_a" and "series_b", and the formulation registers
# one parameter per series under meta keys "a" and "b". A third, unkeyed parameter
# reads the series named by the model's time_series_names.
struct TestMetaParameter <: PSI.TimeSeriesParameter end
struct TestMetaDefaultParameter <: PSI.TimeSeriesParameter end
struct TestMetaDispatch <: PSI.AbstractThermalDispatchFormulation end

const META_TEST_START = DateTime("2024-01-01T00:00:00")
const META_TEST_DEVICES = ("gen_a", "gen_b")

PSI.get_multiplier_value(
    ::Union{TestMetaParameter, TestMetaDefaultParameter},
    ::PSY.ThermalStandard,
    ::TestMetaDispatch,
) = 1.0

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
    return
end

function PSI.construct_device!(
    container::PSI.OptimizationContainer,
    sys::PSY.System,
    ::PSI.ModelConstructStage,
    model::PSI.DeviceModel{PSY.ThermalStandard, TestMetaDispatch},
    network_model::PSI.NetworkModel{<:PM.AbstractActivePowerModel},
)
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
    end
    PSY.transform_single_time_series!(sys, Hour(1), Hour(1))
    return sys
end

function _meta_test_template()
    template = ProblemTemplate(NetworkModel(CopperPlatePowerModel))
    set_device_model!(
        template,
        DeviceModel(
            ThermalStandard,
            TestMetaDispatch;
            time_series_names = Dict(TestMetaDefaultParameter => "series_a"),
        ),
    )
    return template
end

function _meta_test_standalone(initial_time, output_dir)
    model = DecisionModel(
        _meta_test_template(),
        _meta_test_system();
        name = "standalone",
        optimizer = HiGHS_optimizer,
        initial_time = initial_time,
    )
    @test build!(model; output_dir = output_dir) == PSI.ModelBuildStatus.BUILT
    return model
end

function _meta_test_simulation(output_dir)
    model = DecisionModel(
        _meta_test_template(),
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
    return model
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
    model = _meta_test_simulation(output_dir)
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
