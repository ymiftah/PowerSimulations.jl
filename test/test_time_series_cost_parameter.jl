# Toy formulations for TimeSeriesCostParameter. gen_a carries "price" series (or two of them),
# its linear objective coefficient is `COST_TEST_MULT * price * scale` on ActivePowerVariable.
# gen_b is the expensive backup at COST_TEST_BACKUP_COST per pu. The band variant has a second
# variable `x` in [0, 1] driven by the band-2 coefficient of the parameter.
struct TestCostParameter <: PSI.TimeSeriesCostParameter end
struct TestBandCostParameter <: PSI.TimeSeriesCostParameter end

# C selects the variant: :single, :negative, :two (meta "a" and "b"), :band.
struct CostTestDispatch{C} <: PSI.AbstractThermalDispatchFormulation end

const COST_TEST_MULT = 2.0
const COST_TEST_BACKUP_COST = 50.0
const COST_TEST_BASE = 100.0
const COST_TEST_START = DateTime("2024-01-01T00:00:00")
const COST_TEST_BAND_VARS = Dict{Tuple{String, Int}, JuMP.VariableRef}()

PSI.get_multiplier_value(
    ::TestCostParameter,
    ::PSY.ThermalStandard,
    ::CostTestDispatch,
) = COST_TEST_MULT
PSI.get_multiplier_value(
    ::TestBandCostParameter,
    ::PSY.ThermalStandard,
    ::CostTestDispatch,
) = COST_TEST_MULT

# Band 1 is the series value, band 2 is twice the series value.
PSI._unwrap_for_param(::TestBandCostParameter, ts_elem::Float64, ::Any) =
    [ts_elem, 2.0 * ts_elem]
PSI.calc_additional_axes(
    ::PSI.OptimizationContainer,
    ::TestBandCostParameter,
    ::Union{Vector{D}, IS.FlattenIteratorWrapper{D}},
    ::PSI.DeviceModel{D, W},
) where {D <: PSY.Component, W <: PSI.AbstractDeviceFormulation} = (["band_1", "band_2"],)

function _cost_test_scale(::CostTestDispatch{:negative})
    return -1.0
end
_cost_test_scale(::CostTestDispatch) = 1.0

function PSI.construct_device!(
    container::PSI.OptimizationContainer,
    sys::PSY.System,
    ::PSI.ArgumentConstructStage,
    model::PSI.DeviceModel{PSY.ThermalStandard, F},
    network_model::PSI.NetworkModel{<:PM.AbstractActivePowerModel},
) where {F <: CostTestDispatch}
    devices = PSI.get_available_components(model, sys)
    PSI.add_variables!(container, PSI.ActivePowerVariable, devices, F())
    if F <: CostTestDispatch{:two}
        PSI.add_parameters!(
            container,
            TestCostParameter,
            devices,
            model;
            meta = "a",
            time_series_name = "price_a",
        )
        PSI.add_parameters!(
            container,
            TestCostParameter,
            devices,
            model;
            meta = "b",
            time_series_name = "price_b",
        )
    elseif F <: CostTestDispatch{:band}
        PSI.add_parameters!(
            container,
            TestBandCostParameter,
            devices,
            model;
            time_series_name = "price",
        )
    else
        PSI.add_parameters!(
            container,
            TestCostParameter,
            devices,
            model;
            time_series_name = "price",
        )
    end
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
) where {F <: CostTestDispatch}
    devices = PSI.get_available_components(model, sys)
    time_steps = PSI.get_time_steps(container)
    jump_model = PSI.get_jump_model(container)
    p = PSI.get_variable(container, PSI.ActivePowerVariable(), PSY.ThermalStandard)
    if F <: CostTestDispatch{:two}
        for meta in ("a", "b")
            key = PSI.ParameterKey(TestCostParameter, PSY.ThermalStandard, meta)
            names = collect(
                PSI.get_component_names(PSI.get_parameter_attributes(container, key)),
            )
            for name in names, t in time_steps
                PSI.add_cost_target!(
                    container,
                    TestCostParameter(),
                    PSY.ThermalStandard,
                    name,
                    (),
                    t,
                    p[name, t],
                    1.0;
                    meta = meta,
                )
            end
        end
    elseif F <: CostTestDispatch{:band}
        key = PSI.ParameterKey(TestBandCostParameter, PSY.ThermalStandard)
        names =
            collect(PSI.get_component_names(PSI.get_parameter_attributes(container, key)))
        empty!(COST_TEST_BAND_VARS)
        for name in names, t in time_steps
            x = JuMP.@variable(jump_model, lower_bound = 0.0, upper_bound = 1.0)
            COST_TEST_BAND_VARS[(name, t)] = x
            PSI.add_cost_target!(
                container,
                key,
                name,
                (1,),
                t,
                p[name, t],
                1.0,
            )
            PSI.add_cost_target!(
                container,
                key,
                name,
                (2,),
                t,
                x,
                -1.0,
            )
        end
    else
        key = PSI.ParameterKey(TestCostParameter, PSY.ThermalStandard)
        names =
            collect(PSI.get_component_names(PSI.get_parameter_attributes(container, key)))
        scale = _cost_test_scale(F())
        for name in names, t in time_steps
            PSI.add_cost_target!(
                container,
                key,
                name,
                (),
                t,
                p[name, t],
                scale,
            )
        end
    end
    # Base cost of the backup unit only; gen_a is priced by the parameter alone.
    for d in devices, t in time_steps
        name = PSY.get_name(d)
        if name == "gen_b"
            PSI.add_to_objective_invariant_expression!(
                container,
                COST_TEST_BACKUP_COST * p[name, t],
            )
        end
    end
    return
end

function _cost_test_system(series::Dict{String, Vector{Float64}})
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
    n = length(first(values(series)))
    timestamps = range(COST_TEST_START; step = Hour(1), length = n)
    PSY.add_time_series!(
        sys,
        load,
        PSY.SingleTimeSeries("max_active_power", TimeArray(timestamps, ones(n))),
    )
    gen_a = PSY.get_component(PSY.ThermalStandard, sys, "gen_a")
    for (ts_name, values) in series
        PSY.add_time_series!(
            sys,
            gen_a,
            PSY.SingleTimeSeries(ts_name, TimeArray(timestamps, values)),
        )
    end
    PSY.transform_single_time_series!(sys, Hour(1), Hour(1))
    return sys
end

function _cost_test_template(formulation)
    template = ProblemTemplate(
        NetworkModel(CopperPlatePowerModel; duals = [CopperPlateBalanceConstraint]),
    )
    set_device_model!(template, DeviceModel(ThermalStandard, formulation))
    set_device_model!(template, PowerLoad, StaticPowerLoad)
    return template
end

# Expected linear coefficients of gen_a's power (and of the band variable x, if any) at
# `step`, plus the value x takes at its bound in the solution.
function _cost_test_expected(::CostTestDispatch{:single}, series, step)
    return (p = COST_TEST_MULT * series["price"][step], x = 0.0, x_value = 0.0)
end
function _cost_test_expected(::CostTestDispatch{:negative}, series, step)
    return (p = -COST_TEST_MULT * series["price"][step], x = 0.0, x_value = 0.0)
end
function _cost_test_expected(::CostTestDispatch{:two}, series, step)
    total = series["price_a"][step] + series["price_b"][step]
    return (p = COST_TEST_MULT * total, x = 0.0, x_value = 0.0)
end
function _cost_test_expected(::CostTestDispatch{:band}, series, step)
    price = series["price"][step]
    return (p = COST_TEST_MULT * price, x = -COST_TEST_MULT * 2.0 * price, x_value = 1.0)
end

_cost_test_series(::CostTestDispatch{:two}) = Dict(
    "price_a" => [1.0, 2.0, 4.0],
    "price_b" => [0.5, 0.25, 1.5],
)
_cost_test_series(::CostTestDispatch{:zero}) = Dict("price" => [2.0, 0.0, 4.0])
_cost_test_series(::CostTestDispatch) = Dict("price" => [1.0, 2.0, 4.0])

function _cost_test_model(formulation, series; name, initial_time, rebuild_model = false)
    return DecisionModel(
        _cost_test_template(formulation),
        _cost_test_system(series);
        name = name,
        optimizer = HiGHS_optimizer,
        initial_time = initial_time,
        rebuild_model = rebuild_model,
    )
end

function _check_objective_coefficients(container, expected, x_vars = nothing)
    jump_model = PSI.get_jump_model(container)
    obj = JuMP.objective_function(jump_model)
    p = PSI.get_variable(container, ActivePowerVariable(), ThermalStandard)
    @test isapprox(JuMP.coefficient(obj, p["gen_a", 1]), expected.p; atol = 1e-9)
    if !isnothing(x_vars)
        @test isapprox(JuMP.coefficient(obj, x_vars[("gen_a", 1)]), expected.x; atol = 1e-9)
    end
    return
end

function _check_cost_simulation(C::Symbol, series)
    formulation = CostTestDispatch{C}
    output_dir = mktempdir(; cleanup = true)
    n_steps = length(first(values(series)))
    model =
        _cost_test_model(formulation, series; name = "ED", initial_time = COST_TEST_START)
    sim = Simulation(;
        name = "cost_test_$C",
        steps = n_steps,
        models = SimulationModels(; decision_models = [model]),
        sequence = SimulationSequence(;
            models = SimulationModels(; decision_models = [model]),
            ini_cond_chronology = InterProblemChronology(),
        ),
        initial_time = COST_TEST_START,
        simulation_folder = output_dir,
    )
    @test build!(sim) == PSI.SimulationBuildStatus.BUILT
    @test execute!(sim) == PSI.RunStatus.SUCCESSFULLY_FINALIZED

    # The model keeps the last solved step, so its coefficients reflect the final price.
    container = PSI.get_optimization_container(model)
    x_vars = C == :band ? COST_TEST_BAND_VARS : nothing
    expected_last = _cost_test_expected(formulation(), series, n_steps)
    _check_objective_coefficients(container, expected_last, x_vars)

    results = SimulationResults(sim)
    res = get_decision_problem_results(results, "ED")
    power = read_realized_variable(
        res,
        ActivePowerVariable,
        ThermalStandard;
        table_format = TableFormat.WIDE,
    )
    stats = read_optimizer_stats(res)
    for step in 1:n_steps
        expected = _cost_test_expected(formulation(), series, step)
        pa = power[step, "gen_a"] / COST_TEST_BASE
        pb = power[step, "gen_b"] / COST_TEST_BASE
        # gen_a supplies the 1 pu load because it is cheaper than gen_b at every step.
        @test isapprox(pa, 1.0; atol = 1e-6)
        @test isapprox(pb, 0.0; atol = 1e-6)
        objective =
            expected.p * pa + expected.x * expected.x_value + COST_TEST_BACKUP_COST * pb
        @test isapprox(stats[step, "objective_value"], objective; atol = 1e-6)
    end
    return
end

@testset "TimeSeriesCostParameter objective coefficients follow the series" begin
    _check_cost_simulation(:single, _cost_test_series(CostTestDispatch{:single}()))
end

@testset "TimeSeriesCostParameter negative scale flips the contribution" begin
    _check_cost_simulation(:negative, _cost_test_series(CostTestDispatch{:negative}()))
end

@testset "TimeSeriesCostParameter targets on one variable add up" begin
    _check_cost_simulation(:two, _cost_test_series(CostTestDispatch{:two}()))
end

@testset "TimeSeriesCostParameter nonzero, zero, nonzero leaves no stale coefficient" begin
    _check_cost_simulation(:single, _cost_test_series(CostTestDispatch{:zero}()))
end

@testset "TimeSeriesCostParameter band axis targets two variables per device" begin
    _check_cost_simulation(:band, _cost_test_series(CostTestDispatch{:band}()))
end

@testset "add_cost_target! build sets the coefficient at the initial time" begin
    output_dir = mktempdir(; cleanup = true)
    series = _cost_test_series(CostTestDispatch{:single}())
    model = _cost_test_model(
        CostTestDispatch{:single},
        series;
        name = "standalone",
        initial_time = COST_TEST_START,
    )
    @test build!(model; output_dir = output_dir) == PSI.ModelBuildStatus.BUILT
    container = PSI.get_optimization_container(model)
    key = PSI.ParameterKey(TestCostParameter, ThermalStandard)
    @test eltype(PSI.get_parameter_array(container, key)) == Float64
    _check_objective_coefficients(
        container,
        _cost_test_expected(CostTestDispatch{:single}(), series, 1),
    )
end

@testset "add_cost_target! rejects rebuild_model" begin
    output_dir = mktempdir(; cleanup = true)
    model = _cost_test_model(
        CostTestDispatch{:single},
        _cost_test_series(CostTestDispatch{:single}());
        name = "rebuild",
        initial_time = COST_TEST_START,
        rebuild_model = true,
    )
    @test build!(model; output_dir = output_dir, console_level = Logging.AboveMaxLevel) ==
          PSI.ModelBuildStatus.FAILED
end
