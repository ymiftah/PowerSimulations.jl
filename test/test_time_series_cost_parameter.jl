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

function _cost_test_system(
    series::Dict{String, Vector{Float64}};
    horizon = Hour(1),
    shared_to = String[],
)
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
    shared_names = [name for name in shared_to]
    for (ts_name, values) in series
        # The same series object is attached to every listed generator.
        ts = PSY.SingleTimeSeries(ts_name, TimeArray(timestamps, values))
        PSY.add_time_series!(sys, gen_a, ts)
        for name in shared_names
            PSY.add_time_series!(sys, PSY.get_component(PSY.ThermalStandard, sys, name), ts)
        end
    end
    PSY.transform_single_time_series!(sys, horizon, Hour(1))
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

# Both generators share one price series object, so they share one parameter row but keep
# their own multiplier rows.
struct SharedCostDispatch <: PSI.AbstractThermalDispatchFormulation end
PSI.get_multiplier_value(
    ::TestCostParameter,
    d::PSY.ThermalStandard,
    ::SharedCostDispatch,
) = PSY.get_name(d) == "gen_a" ? 2.0 : 0.5

function PSI.construct_device!(
    container::PSI.OptimizationContainer,
    sys::PSY.System,
    ::PSI.ArgumentConstructStage,
    model::PSI.DeviceModel{PSY.ThermalStandard, SharedCostDispatch},
    network_model::PSI.NetworkModel{<:PM.AbstractActivePowerModel},
)
    devices = PSI.get_available_components(model, sys)
    PSI.add_variables!(container, PSI.ActivePowerVariable, devices, SharedCostDispatch())
    PSI.add_parameters!(
        container,
        TestCostParameter,
        devices,
        model;
        time_series_name = "price",
    )
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
    ::PSY.System,
    ::PSI.ModelConstructStage,
    model::PSI.DeviceModel{PSY.ThermalStandard, SharedCostDispatch},
    ::PSI.NetworkModel{<:PM.AbstractActivePowerModel},
)
    time_steps = PSI.get_time_steps(container)
    p = PSI.get_variable(container, PSI.ActivePowerVariable(), PSY.ThermalStandard)
    key = PSI.ParameterKey(TestCostParameter, PSY.ThermalStandard)
    names = collect(PSI.get_component_names(PSI.get_parameter_attributes(container, key)))
    for name in names, t in time_steps
        PSI.add_cost_target!(container, key, name, (), t, p[name, t], 1.0)
    end
    return
end

@testset "Shared series row keeps per-device multipliers" begin
    prices = [1.0, 2.0, 4.0, 3.0, 5.0]
    sys = _cost_test_system(
        Dict("price" => prices);
        horizon = Hour(3),
        shared_to = ["gen_b"],
    )
    model = DecisionModel(
        _cost_test_template(SharedCostDispatch),
        sys;
        name = "ED",
        optimizer = HiGHS_optimizer,
        initial_time = COST_TEST_START,
    )
    sim = Simulation(;
        name = "shared_cost",
        steps = 3,
        models = SimulationModels(; decision_models = [model]),
        sequence = SimulationSequence(;
            models = SimulationModels(; decision_models = [model]),
            ini_cond_chronology = InterProblemChronology(),
        ),
        initial_time = COST_TEST_START,
        simulation_folder = mktempdir(; cleanup = true),
    )
    @test build!(sim) == PSI.SimulationBuildStatus.BUILT
    @test execute!(sim) == PSI.RunStatus.SUCCESSFULLY_FINALIZED

    key = PSI.ParameterKey(TestCostParameter, ThermalStandard)
    container = PSI.get_optimization_container(model)
    @test length(axes(PSI.get_parameter_array(container, key), 1)) == 1
    # The final step starts at hour 3, so horizon hours 3 to 5 are the cost coefficients.
    obj = JuMP.objective_function(PSI.get_jump_model(container))
    p = PSI.get_variable(container, ActivePowerVariable(), ThermalStandard)
    for t in 1:3
        @test isapprox(
            JuMP.coefficient(obj, p["gen_a", t]),
            2.0 * prices[2 + t];
            atol = 1e-9,
        )
        @test isapprox(
            JuMP.coefficient(obj, p["gen_b", t]),
            0.5 * prices[2 + t];
            atol = 1e-9,
        )
    end

    res = get_decision_problem_results(SimulationResults(sim), "ED")
    stats = read_optimizer_stats(res)
    power = read_realized_variable(
        res,
        ActivePowerVariable,
        ThermalStandard;
        table_format = TableFormat.WIDE,
    )
    # gen_b is the cheaper unit at every step, so it carries the 1 pu load.
    for step in 1:3
        @test isapprox(power[step, "gen_b"], 100.0; atol = 1e-6)
        @test isapprox(power[step, "gen_a"], 0.0; atol = 1e-6)
        expected = sum(0.5 * prices[step + t - 1] for t in 1:3)
        @test isapprox(stats[step, "objective_value"], expected; atol = 1e-6)
    end
end

# Service-owned cost parameter on two reserves with distinct series. Their up and down
# slacks are forced to 0.5 and 0.25 so both directions of the coefficient are used.
struct TestPenaltyParameter <: PSI.TimeSeriesCostParameter end
struct TestPenaltyService <: PSI.AbstractReservesFormulation end
struct ServiceTestDispatch <: PSI.AbstractThermalDispatchFormulation end
const SERVICE_TEST_SLACKS = Dict{
    Tuple{String, Int},
    NamedTuple{(:up, :down), Tuple{JuMP.VariableRef, JuMP.VariableRef}},
}()

PSI.get_multiplier_value(::TestPenaltyParameter, ::PSY.Service, ::TestPenaltyService) = 1.0

function PSI.construct_device!(
    container::PSI.OptimizationContainer,
    sys::PSY.System,
    ::PSI.ArgumentConstructStage,
    model::PSI.DeviceModel{PSY.ThermalStandard, ServiceTestDispatch},
    network_model::PSI.NetworkModel{<:PM.AbstractActivePowerModel},
)
    devices = PSI.get_available_components(model, sys)
    PSI.add_variables!(container, PSI.ActivePowerVariable, devices, ServiceTestDispatch())
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
    ::PSI.OptimizationContainer,
    ::PSY.System,
    ::PSI.ModelConstructStage,
    ::PSI.DeviceModel{PSY.ThermalStandard, ServiceTestDispatch},
    ::PSI.NetworkModel{<:PM.AbstractActivePowerModel},
)
    return
end

function PSI.construct_service!(
    container::PSI.OptimizationContainer,
    sys::PSY.System,
    ::PSI.ArgumentConstructStage,
    model::PSI.ServiceModel{SR, TestPenaltyService},
    ::Dict{Symbol, PSI.DeviceModel},
    ::Set{<:DataType},
    ::PSI.NetworkModel{<:PM.AbstractPowerModel},
) where {SR <: PSY.Reserve}
    service = PSY.get_component(SR, sys, PSI.get_service_name(model))
    PSI.add_parameters!(container, TestPenaltyParameter, service, model)
    return
end

function PSI.construct_service!(
    container::PSI.OptimizationContainer,
    ::PSY.System,
    ::PSI.ModelConstructStage,
    model::PSI.ServiceModel{SR, TestPenaltyService},
    ::Dict{Symbol, PSI.DeviceModel},
    ::Set{<:DataType},
    ::PSI.NetworkModel{<:PM.AbstractPowerModel},
) where {SR <: PSY.Reserve}
    name = PSI.get_service_name(model)
    jump_model = PSI.get_jump_model(container)
    for t in PSI.get_time_steps(container)
        up = JuMP.@variable(jump_model, lower_bound = 0.5, upper_bound = 1.0)
        down = JuMP.@variable(jump_model, lower_bound = 0.0, upper_bound = 0.25)
        SERVICE_TEST_SLACKS[(name, t)] = (up = up, down = down)
        PSI.add_cost_target!(container, TestPenaltyParameter(), SR, name, (), t, up, 1.0)
        PSI.add_cost_target!(container, TestPenaltyParameter(), SR, name, (), t, down, -1.0)
    end
    return
end

@testset "Service-owned cost parameter targets each service's own slacks" begin
    penalties = Dict("R1" => [1.0, 0.0, 3.0], "R2" => [2.0, 1.0, 0.5])
    sys = _cost_test_system(Dict("price" => ones(3)))
    for name in ("R1", "R2")
        reserve = PSY.ConstantReserve{PSY.ReserveUp}(name, true, 30.0, 0.5)
        PSY.add_service!(
            sys,
            reserve,
            PSY.get_components(PSY.ThermalStandard, sys),
        )
        PSY.add_time_series!(
            sys,
            reserve,
            PSY.SingleTimeSeries(
                "penalty",
                TimeArray(
                    range(COST_TEST_START; step = Hour(1), length = 3),
                    penalties[name],
                ),
            ),
        )
    end
    PSY.transform_single_time_series!(sys, Hour(1), Hour(1))

    template = ProblemTemplate(
        NetworkModel(CopperPlatePowerModel; duals = [CopperPlateBalanceConstraint]),
    )
    set_device_model!(template, DeviceModel(ThermalStandard, ServiceTestDispatch))
    set_device_model!(template, PowerLoad, StaticPowerLoad)
    for name in ("R1", "R2")
        set_service_model!(
            template,
            ServiceModel(
                ConstantReserve{ReserveUp},
                TestPenaltyService,
                name;
                time_series_names = Dict(TestPenaltyParameter => "penalty"),
            ),
        )
    end
    model = DecisionModel(
        template,
        sys;
        name = "ED",
        optimizer = HiGHS_optimizer,
        initial_time = COST_TEST_START,
    )
    sim = Simulation(;
        name = "service_cost",
        steps = 3,
        models = SimulationModels(; decision_models = [model]),
        sequence = SimulationSequence(;
            models = SimulationModels(; decision_models = [model]),
            ini_cond_chronology = InterProblemChronology(),
        ),
        initial_time = COST_TEST_START,
        simulation_folder = mktempdir(; cleanup = true),
    )
    @test build!(sim) == PSI.SimulationBuildStatus.BUILT
    @test execute!(sim) == PSI.RunStatus.SUCCESSFULLY_FINALIZED

    # Per step the net contribution of one service is price * (0.5 - 0.25).
    res = get_decision_problem_results(SimulationResults(sim), "ED")
    stats = read_optimizer_stats(res)
    for step in 1:3
        expected = sum(0.25 * penalties[name][step] for name in ("R1", "R2"))
        @test isapprox(stats[step, "objective_value"], expected; atol = 1e-6)
    end

    # The final step's coefficients come from each service's own series.
    container = PSI.get_optimization_container(model)
    obj = JuMP.objective_function(PSI.get_jump_model(container))
    for name in ("R1", "R2")
        up = SERVICE_TEST_SLACKS[(name, 1)].up
        down = SERVICE_TEST_SLACKS[(name, 1)].down
        @test isapprox(JuMP.coefficient(obj, up), penalties[name][3]; atol = 1e-9)
        @test isapprox(JuMP.coefficient(obj, down), -penalties[name][3]; atol = 1e-9)
    end
end

@testset "add_cost_target! rejects wrong indices and time steps" begin
    series = _cost_test_series(CostTestDispatch{:single}())
    model = _cost_test_model(
        CostTestDispatch{:single},
        series;
        name = "bad_args",
        initial_time = COST_TEST_START,
    )
    @test build!(model; output_dir = mktempdir(; cleanup = true)) ==
          PSI.ModelBuildStatus.BUILT
    container = PSI.get_optimization_container(model)
    key = PSI.ParameterKey(TestCostParameter, ThermalStandard)
    p = PSI.get_variable(container, ActivePowerVariable(), ThermalStandard)
    # The parameter has no additional axis, so an index is an error.
    @test_throws ErrorException PSI.add_cost_target!(
        container, key, "gen_a", (1,), 1, p["gen_a", 1], 1.0,
    )
    # Two indices are outside the supported shape.
    @test_throws MethodError PSI.add_cost_target!(
        container, key, "gen_a", (1, 1), 1, p["gen_a", 1], 1.0,
    )
    # The model has one time step.
    @test_throws ErrorException PSI.add_cost_target!(
        container, key, "gen_a", (), 2, p["gen_a", 1], 1.0,
    )
end
