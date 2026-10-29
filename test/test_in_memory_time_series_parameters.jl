# Parameter updates with time series stored in memory, where the model time-series cache is
# disabled. Devices "gen_a" and "gen_b" carry the forecast "series"; "gen_none" carries no
# series and is therefore not registered. The full parameter registers every device, the
# subset parameter only "gen_a".
struct TestInMemoryParameter <: PSI.TimeSeriesParameter end
struct TestInMemorySubsetParameter <: PSI.TimeSeriesParameter end
struct TestInMemoryDispatch <: PSI.AbstractThermalDispatchFormulation end

const IN_MEMORY_TEST_START = DateTime("2024-01-01T00:00:00")
const IN_MEMORY_TEST_HORIZON = 3
const IN_MEMORY_TEST_STEPS = 4
const IN_MEMORY_TEST_SERIES = Dict("gen_a" => 1.0, "gen_b" => 7.0)

PSI.get_multiplier_value(
    ::Union{TestInMemoryParameter, TestInMemorySubsetParameter},
    ::PSY.ThermalStandard,
    ::TestInMemoryDispatch,
) = 1.0

function PSI.construct_device!(
    container::PSI.OptimizationContainer,
    sys::PSY.System,
    ::PSI.ArgumentConstructStage,
    model::PSI.DeviceModel{PSY.ThermalStandard, TestInMemoryDispatch},
    network_model::PSI.NetworkModel{<:PM.AbstractActivePowerModel},
)
    devices = PSI.get_available_components(model, sys)
    PSI.add_variables!(container, PSI.ActivePowerVariable, devices, TestInMemoryDispatch())
    PSI.add_parameters!(
        container,
        TestInMemoryParameter,
        devices,
        model;
        meta = "all",
        time_series_name = "series",
    )
    subset = [d for d in devices if PSY.get_name(d) == "gen_a"]
    PSI.add_parameters!(
        container,
        TestInMemorySubsetParameter,
        subset,
        model;
        meta = "subset",
        time_series_name = "series",
    )
    return
end

function PSI.construct_device!(
    ::PSI.OptimizationContainer,
    ::PSY.System,
    ::PSI.ModelConstructStage,
    ::PSI.DeviceModel{PSY.ThermalStandard, TestInMemoryDispatch},
    ::PSI.NetworkModel{<:PM.AbstractActivePowerModel},
)
    return
end

# Values differ per window and per hour, so a window read at the wrong step is detected.
_in_memory_test_value(offset, hour) = offset + 10.0 * hour + 0.25 * hour^2

function _in_memory_test_system(; single_time_series::Bool)
    sys = PSY.System(100.0; time_series_in_memory = true)
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
    for name in ("gen_a", "gen_b", "gen_none")
        gen = PSY.ThermalStandard(;
            name = name,
            available = true,
            status = true,
            bus = bus,
            active_power = 0.0,
            reactive_power = 0.0,
            rating = 2.0,
            active_power_limits = (min = 0.0, max = 2.0),
            reactive_power_limits = nothing,
            ramp_limits = nothing,
            operation_cost = PSY.ThermalGenerationCost(nothing),
            base_power = 100.0,
            prime_mover_type = PSY.PrimeMovers.ST,
            fuel = PSY.ThermalFuels.COAL,
        )
        PSY.add_component!(sys, gen)
    end
    resolution = Hour(1)
    n_windows = IN_MEMORY_TEST_STEPS + 1
    for (name, offset) in IN_MEMORY_TEST_SERIES
        gen = PSY.get_component(PSY.ThermalStandard, sys, name)
        # The multiplier changes the values if scaling factors were applied.
        if single_time_series
            len = n_windows + IN_MEMORY_TEST_HORIZON - 1
            timestamps = range(IN_MEMORY_TEST_START; step = resolution, length = len)
            ts = PSY.SingleTimeSeries(
                "series",
                TimeArray(timestamps, [_in_memory_test_value(offset, h) for h in 1:len]);
                scaling_factor_multiplier = PSY.get_rating,
            )
        else
            data = SortedDict(
                IN_MEMORY_TEST_START + w * resolution => [
                    _in_memory_test_value(offset + 100.0 * w, h) for
                    h in 1:IN_MEMORY_TEST_HORIZON
                ] for w in 0:(n_windows - 1)
            )
            ts = PSY.Deterministic(;
                name = "series",
                data = data,
                resolution = resolution,
                scaling_factor_multiplier = PSY.get_rating,
            )
        end
        PSY.add_time_series!(sys, gen, ts)
    end
    if single_time_series
        PSY.transform_single_time_series!(sys, Hour(IN_MEMORY_TEST_HORIZON), resolution)
    end
    return sys
end

function _in_memory_test_simulation(output_dir; single_time_series::Bool)
    sys = _in_memory_test_system(; single_time_series = single_time_series)
    @test PSY.stores_time_series_in_memory(sys)
    template = ProblemTemplate(NetworkModel(CopperPlatePowerModel))
    set_device_model!(template, DeviceModel(ThermalStandard, TestInMemoryDispatch))
    model = DecisionModel(
        template,
        sys;
        name = "ED",
        optimizer = HiGHS_optimizer,
        horizon = Hour(IN_MEMORY_TEST_HORIZON),
        interval = Hour(1),
    )
    sim = Simulation(;
        name = "in_memory_parameter_test",
        steps = IN_MEMORY_TEST_STEPS,
        models = SimulationModels(; decision_models = [model]),
        sequence = SimulationSequence(;
            models = SimulationModels(; decision_models = [model]),
            ini_cond_chronology = InterProblemChronology(),
        ),
        initial_time = IN_MEMORY_TEST_START,
        simulation_folder = output_dir,
    )
    @test build!(sim) == PSI.SimulationBuildStatus.BUILT
    @test !PSI.use_time_series_cache(PSI.get_settings(model))
    @test execute!(sim) == PSI.RunStatus.SUCCESSFULLY_FINALIZED
    return sim, model
end

# Compares the stored parameter values of every step with a direct read of the system.
function _test_in_memory_parameter_steps(res, sys, key_name, expected_names)
    values = read_parameter(res, key_name; table_format = TableFormat.WIDE)
    @test length(values) == IN_MEMORY_TEST_STEPS
    for (step, (step_time, df)) in enumerate(values)
        @test step_time == IN_MEMORY_TEST_START + Hour(step - 1)
        @test sort(setdiff(names(df), ["DateTime"])) == sort(expected_names)
        for name in expected_names
            gen = PSY.get_component(PSY.ThermalStandard, sys, name)
            expected = PSY.get_time_series_values(
                PSY.Deterministic,
                gen,
                "series";
                start_time = step_time,
                len = IN_MEMORY_TEST_HORIZON,
                ignore_scaling_factors = true,
            )
            @test df[!, name] == expected
        end
    end
    return
end

@testset "In-memory time-series parameters update every step (Deterministic)" begin
    output_dir = mktempdir(; cleanup = true)
    sim, model = _in_memory_test_simulation(output_dir; single_time_series = false)
    sys = PSI.get_system(model)
    res = get_decision_problem_results(SimulationResults(sim), "ED")
    _test_in_memory_parameter_steps(
        res,
        sys,
        "TestInMemoryParameter__ThermalStandard__all",
        ["gen_a", "gen_b"],
    )
    _test_in_memory_parameter_steps(
        res,
        sys,
        "TestInMemorySubsetParameter__ThermalStandard__subset",
        ["gen_a"],
    )
    # One metadata entry per device with the series, shared by both parameters.
    cache = PSI.get_time_series_metadata_cache(PSI.get_optimization_container(model))
    @test length(cache) == 2
    gen_none = PSY.get_component(PSY.ThermalStandard, sys, "gen_none")
    @test all(k -> k.component_uuid != IS.get_uuid(gen_none), keys(cache))
end

@testset "In-memory time-series parameters update every step (DeterministicSingleTimeSeries)" begin
    output_dir = mktempdir(; cleanup = true)
    sim, model = _in_memory_test_simulation(output_dir; single_time_series = true)
    sys = PSI.get_system(model)
    res = get_decision_problem_results(SimulationResults(sim), "ED")
    _test_in_memory_parameter_steps(
        res,
        sys,
        "TestInMemoryParameter__ThermalStandard__all",
        ["gen_a", "gen_b"],
    )
    _test_in_memory_parameter_steps(
        res,
        sys,
        "TestInMemorySubsetParameter__ThermalStandard__subset",
        ["gen_a"],
    )
end

@testset "In-memory SingleTimeSeries parameters update every EmulationModel step" begin
    sys = PSB.build_system(
        PSITestSystems,
        "c_sys5_uc";
        add_single_time_series = true,
        time_series_in_memory = true,
        force_build = true,
    )
    @test PSY.stores_time_series_in_memory(sys)
    template = get_thermal_dispatch_template_network()
    model = EmulationModel(template, sys; optimizer = HiGHS_optimizer)
    executions = 5
    @test build!(
        model;
        executions = executions,
        output_dir = mktempdir(; cleanup = true),
    ) ==
          PSI.ModelBuildStatus.BUILT
    @test run!(model) == PSI.RunStatus.SUCCESSFULLY_FINALIZED
    container = PSI.get_optimization_container(model)
    key = PSI.ParameterKey(ActivePowerTimeSeriesParameter, PowerLoad)
    multipliers = PSI.get_multiplier_array(PSI.get_parameter(container, key))
    # One row per execution and load, in execution order.
    values = read_parameter(OptimizationProblemResults(model), key)
    initial_time = PSI.get_initial_time(model)
    for load in PSY.get_components(PowerLoad, sys)
        name = PSY.get_name(load)
        expected = PSY.get_time_series_values(
            PSY.SingleTimeSeries,
            load,
            "max_active_power";
            start_time = initial_time,
            len = executions,
            ignore_scaling_factors = true,
        )
        # Stored results are in natural units.
        @test values[values.name .== name, :value] ==
              expected .* multipliers[name, 1] .* PSY.get_base_power(sys)
    end
end

@testset "Reduced branch rating on EmulationModel reads the parallel line that owns the series" begin
    # The series sits on one of two parallel lines; the other line is registered through the
    # reduction entry and must take its values from the owner. Both lines are tried as owner.
    for series_on_copy in (false, true)
        sys = PSB.build_system(
            PSITestSystems,
            "c_sys5";
            time_series_in_memory = true,
            force_build = true,
        )
        line_names_before = Set(PSY.get_name.(PSY.get_components(PSY.Line, sys)))
        add_equivalent_ac_transmission_with_parallel_circuits!(
            sys,
            PSY.get_component(PSY.Line, sys, "1"),
            PSY.Line,
        )
        copy_name = only(
            setdiff(PSY.get_name.(PSY.get_components(PSY.Line, sys)), line_names_before),
        )
        owner_name = series_on_copy ? copy_name : "1"
        owner = PSY.get_component(PSY.Line, sys, owner_name)
        executions = 3
        timestamps = range(IN_MEMORY_TEST_START; step = Hour(1), length = 4)
        ts = PSY.SingleTimeSeries(
            "branch_rating",
            TimeArray(timestamps, [0.9, 0.8, 1.0, 0.7]);
            scaling_factor_multiplier = PSY.get_rating,
        )
        PSY.add_time_series!(sys, owner, ts)

        template = ProblemTemplate(
            NetworkModel(PTDFPowerModel; PTDF_matrix = PTDF(sys)),
        )
        set_device_model!(template, DeviceModel(ThermalStandard, ThermalBasicDispatch))
        set_device_model!(
            template,
            DeviceModel(
                Line,
                StaticBranch;
                time_series_names = Dict(
                    BranchRatingTimeSeriesParameter => "branch_rating",
                ),
            ),
        )
        model = EmulationModel(template, sys; optimizer = HiGHS_optimizer)
        @test build!(
            model;
            executions = executions,
            output_dir = mktempdir(; cleanup = true),
        ) == PSI.ModelBuildStatus.BUILT
        @test run!(model) == PSI.RunStatus.SUCCESSFULLY_FINALIZED

        container = PSI.get_optimization_container(model)
        key = PSI.ParameterKey(BranchRatingTimeSeriesParameter, Line)
        multipliers = PSI.get_multiplier_array(PSI.get_parameter(container, key))
        values = read_parameter(OptimizationProblemResults(model), key)
        initial_time = PSI.get_initial_time(model)
        expected = PSY.get_time_series_values(
            PSY.SingleTimeSeries,
            owner,
            "branch_rating";
            start_time = initial_time,
            len = executions,
            ignore_scaling_factors = true,
        )
        # The parallel pair shares one reduced row, named after the reduction entry.
        reduced_name = only(axes(multipliers)[1])
        # Stored results are in natural units.
        @test values[values.name .== reduced_name, :value] ==
              expected .* multipliers[reduced_name, 1] .* PSY.get_base_power(sys)
    end
end
