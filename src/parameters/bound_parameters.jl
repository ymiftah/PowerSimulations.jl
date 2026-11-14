"""
    add_bound_target!(container, key, name, t, constraint, active)

Register `constraint` as driven by cell `(name, t)` of the [`ConstraintBoundParameter`](@ref)
stored under `key`: the row has the bounds `active = (lo, hi)` where `value * multiplier` is
nonzero and is free (`-Inf`, `Inf`) otherwise. The bounds are set from the current parameter
values, and simulation updates reset them after each parameter update.

# Arguments

  - `container::OptimizationContainer`: container holding the parameter.
  - `key::ParameterKey{T, U}`: key of a parameter with `T <: ConstraintBoundParameter`.
  - `name::String`: component name on the parameter's first axis.
  - `t::Int`: time step.
  - `constraint::JuMP.ConstraintRef`: constraint built as a `MOI.Interval` row, whose bounds
    are replaced.
  - `active::NTuple{2, Float64}`: the bounds of the row while active.
"""
function add_bound_target!(
    container::OptimizationContainer,
    key::ParameterKey{T, U},
    name::String,
    t::Int,
    constraint::JuMP.ConstraintRef,
    active::NTuple{2, Float64},
) where {T <: ConstraintBoundParameter, U <: PSY.Component}
    if get_rebuild_model(get_settings(container))
        error("$T does not support models built with rebuild_model = true")
    end
    param_container = get_parameter(container, key)
    param_array = get_parameter_array(param_container)
    multiplier_array = get_multiplier_array(param_container)
    if !(param_array isa DenseAxisArray) || ndims(param_array) != 2
        error("$T requires a dense (component, time) parameter array")
    end
    if !(1 <= t <= size(param_array, 2))
        error("Time step $t is outside the time steps of $T")
    end
    attributes = get_attributes(param_container)
    param_row = param_array.lookup[1][_get_ts_uuid(attributes, name)]
    multiplier_row = multiplier_array.lookup[1][name]
    target = BoundTarget(
        constraint,
        active[1],
        active[2],
        param_row,
        multiplier_row,
        t,
    )
    push!(get_bound_targets(attributes), target)
    _apply_bound_target!(
        get_parameter_array_data(param_container),
        get_multiplier_array_data(param_container),
        target,
    )
    return
end

function add_bound_target!(
    container::OptimizationContainer,
    ::T,
    ::Type{U},
    name::String,
    t::Int,
    constraint::JuMP.ConstraintRef,
    active::NTuple{2, Float64};
    meta = ISOPT.CONTAINER_KEY_EMPTY_META,
) where {T <: ConstraintBoundParameter, U <: PSY.Component}
    return add_bound_target!(
        container,
        ParameterKey(T, U, meta),
        name,
        t,
        constraint,
        active,
    )
end

function _apply_bound_target!(
    parent_param::Matrix{Float64},
    parent_mult::Matrix{Float64},
    target::BoundTarget,
)
    value =
        parent_param[target.param_row, target.time_step] *
        parent_mult[target.multiplier_row, target.time_step]
    bounds =
        value == 0.0 ? MOI.Interval(-Inf, Inf) :
        MOI.Interval(target.active_lo, target.active_hi)
    MOI.set(JuMP.owner_model(target.constraint), MOI.ConstraintSet(), target.constraint, bounds)
    return
end

function _apply_bound_targets!(param_container::ParameterContainer)
    parent_param = get_parameter_array_data(param_container)
    parent_mult = get_multiplier_array_data(param_container)
    for target in get_bound_targets(get_attributes(param_container))
        _apply_bound_target!(parent_param, parent_mult, target)
    end
    return
end

function update_container_parameter_values!(
    optimization_container::OptimizationContainer,
    model::OperationModel,
    key::ParameterKey{T, U},
    input::DatasetContainer{InMemoryDataset},
) where {T <: ConstraintBoundParameter, U <: PSY.Component}
    param_container = get_parameter(optimization_container, key)
    _update_parameter_values!(
        get_parameter_array(param_container),
        T(),
        get_attributes(param_container),
        U,
        model,
        input,
    )
    _apply_bound_targets!(param_container)
    return
end

function update_container_parameter_values!(
    optimization_container::OptimizationContainer,
    model::OperationModel,
    key::ParameterKey{T, U},
    input::DatasetContainer{InMemoryDataset},
) where {T <: ConstraintBoundParameter, U <: PSY.Service}
    param_container = get_parameter(optimization_container, key)
    service = PSY.get_component(U, get_system(model), key.meta)
    isnothing(service) && error(
        "Service $(U) named '$(key.meta)' is no longer in the system; " *
        "cannot update parameter $(key)",
    )
    _update_parameter_values!(
        get_parameter_array(param_container),
        T(),
        get_attributes(param_container),
        service,
        model,
        input,
    )
    _apply_bound_targets!(param_container)
    return
end
