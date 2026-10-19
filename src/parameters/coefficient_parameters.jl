"""
    add_coefficient_target!(container, key, name, t, constraint, variable, scale)

Register the coefficient of `variable` in `constraint` as driven by cell `(name, t)` of the
[`ConstraintCoefficientParameter`](@ref) stored under `key`, and set it to
`scale * value * multiplier` from the current parameter values. Simulation updates rewrite
the coefficient with the same expression after each parameter update.

# Arguments

  - `container::OptimizationContainer`: container holding the parameter.
  - `key::ParameterKey{T, U}`: key of a parameter with `T <: ConstraintCoefficientParameter`.
  - `name::String`: component name on the parameter's first axis.
  - `t::Int`: time step.
  - `constraint::JuMP.ConstraintRef`: constraint whose coefficient is set.
  - `variable::JuMP.VariableRef`: variable whose coefficient is set.
  - `scale::Float64`: factor applied to the parameter value; the coefficient is the
    normalized one, with every variable term on the left-hand side.
"""
function add_coefficient_target!(
    container::OptimizationContainer,
    key::ParameterKey{T, U},
    name::String,
    t::Int,
    constraint::JuMP.ConstraintRef,
    variable::JuMP.VariableRef,
    scale::Float64,
) where {T <: ConstraintCoefficientParameter, U <: PSY.Component}
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
    target = CoefficientTarget(constraint, variable, scale, param_row, multiplier_row, t)
    push!(get_coefficient_targets(attributes), target)
    _apply_coefficient_target!(
        get_parameter_array_data(param_container),
        get_multiplier_array_data(param_container),
        target,
    )
    return
end

function add_coefficient_target!(
    container::OptimizationContainer,
    ::T,
    ::Type{U},
    name::String,
    t::Int,
    constraint::JuMP.ConstraintRef,
    variable::JuMP.VariableRef,
    scale::Float64;
    meta = ISOPT.CONTAINER_KEY_EMPTY_META,
) where {T <: ConstraintCoefficientParameter, U <: PSY.Component}
    return add_coefficient_target!(
        container,
        ParameterKey(T, U, meta),
        name,
        t,
        constraint,
        variable,
        scale,
    )
end

function _apply_coefficient_target!(
    parent_param::Matrix{Float64},
    parent_mult::Matrix{Float64},
    target::CoefficientTarget,
)
    value =
        parent_param[target.param_row, target.time_step] *
        parent_mult[target.multiplier_row, target.time_step]
    JuMP.set_normalized_coefficient(
        target.constraint,
        target.variable,
        target.scale * value,
    )
    return
end

function _apply_coefficient_targets!(param_container::ParameterContainer)
    parent_param = get_parameter_array_data(param_container)
    parent_mult = get_multiplier_array_data(param_container)
    for target in get_coefficient_targets(get_attributes(param_container))
        _apply_coefficient_target!(parent_param, parent_mult, target)
    end
    return
end

function update_container_parameter_values!(
    optimization_container::OptimizationContainer,
    model::OperationModel,
    key::ParameterKey{T, U},
    input::DatasetContainer{InMemoryDataset},
) where {T <: ConstraintCoefficientParameter, U <: PSY.Component}
    # Note: Do not instantiate a new key here because it might not match the param keys in the container
    # if the keys have strings in the meta fields
    param_container = get_parameter(optimization_container, key)
    _update_parameter_values!(
        get_parameter_array(param_container),
        T(),
        get_attributes(param_container),
        U,
        model,
        input,
    )
    _apply_coefficient_targets!(param_container)
    return
end

function update_container_parameter_values!(
    optimization_container::OptimizationContainer,
    model::OperationModel,
    key::ParameterKey{T, U},
    input::DatasetContainer{InMemoryDataset},
) where {T <: ConstraintCoefficientParameter, U <: PSY.Service}
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
    _apply_coefficient_targets!(param_container)
    return
end
