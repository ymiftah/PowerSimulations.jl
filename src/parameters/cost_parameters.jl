"""
    add_cost_target!(container, key, name, indices, t, variable, scale)

Add `scale * value * multiplier * variable` to the variant objective, where `value` and
`multiplier` are read at `(name, indices..., t)` of the [`TimeSeriesCostParameter`](@ref)
stored under `key`. Simulation updates rebuild the variant terms from the parameter values
after each parameter update.

# Arguments

  - `container::OptimizationContainer`: container holding the parameter.
  - `key::ParameterKey{T, U}`: key of a parameter with `T <: TimeSeriesCostParameter` and
    `U` a component or service type.
  - `name::String`: component or service name on the parameter's first axis.
  - `indices::Tuple{Vararg{Int}}`: integer positions on the additional axes between the
    component and time axes; `()` for a (component, time) parameter.
  - `t::Int`: time step.
  - `variable::JuMP.VariableRef`: variable whose objective coefficient is set.
  - `scale::Float64`: factor applied to the parameter value.
"""
function add_cost_target!(
    container::OptimizationContainer,
    key::ParameterKey{T, U},
    name::String,
    indices::Tuple{Vararg{Int}},
    t::Int,
    variable::JuMP.VariableRef,
    scale::Float64,
) where {T <: TimeSeriesCostParameter, U <: Union{PSY.Component, PSY.Service}}
    if get_rebuild_model(get_settings(container))
        error("$T does not support models built with rebuild_model = true")
    end
    param_container = get_parameter(container, key)
    param_array = get_parameter_array(param_container)
    multiplier_array = get_multiplier_array(param_container)
    if !(param_array isa DenseAxisArray)
        error("$T requires a dense parameter array")
    end
    if ndims(param_array) != 2 + length(indices)
        error(
            "$T with $(ndims(param_array) - 2) additional axes needs $(ndims(param_array) - 2) " *
            "indices, got $(length(indices))",
        )
    end
    if !(1 <= t <= size(param_array, ndims(param_array)))
        error("Time step $t is outside the time steps of $T")
    end
    attributes = get_attributes(param_container)
    param_row = param_array.lookup[1][_get_ts_uuid(attributes, name)]
    multiplier_row = multiplier_array.lookup[1][name]
    parent_param = get_parameter_array_data(param_container)
    parent_mult = get_multiplier_array_data(param_container)
    target = CostTarget(
        variable,
        scale,
        LinearIndices(parent_param)[param_row, indices..., t],
        LinearIndices(parent_mult)[multiplier_row, indices..., t],
    )
    push!(get_cost_targets(attributes), target)
    _add_cost_target_term!(container, parent_param, parent_mult, target)
    return
end

function add_cost_target!(
    container::OptimizationContainer,
    ::T,
    ::Type{U},
    name::String,
    indices::Tuple{Vararg{Int}},
    t::Int,
    variable::JuMP.VariableRef,
    scale::Float64;
    meta = ISOPT.CONTAINER_KEY_EMPTY_META,
) where {T <: TimeSeriesCostParameter, U <: Union{PSY.Component, PSY.Service}}
    return add_cost_target!(
        container,
        ParameterKey(T, U, meta),
        name,
        indices,
        t,
        variable,
        scale,
    )
end

function _add_cost_target_term!(
    container::OptimizationContainer,
    parent_param::Array{Float64},
    parent_mult::Array{Float64},
    target::CostTarget,
)
    coefficient =
        target.scale * parent_param[target.param_index] *
        parent_mult[target.multiplier_index]
    add_to_objective_variant_expression!(container, coefficient * target.variable)
    return
end

function _apply_cost_targets!(
    container::OptimizationContainer,
    param_container::ParameterContainer,
)
    targets = get_cost_targets(get_attributes(param_container))
    parent_param = get_parameter_array_data(param_container)
    parent_mult = get_multiplier_array_data(param_container)
    for target in targets
        _add_cost_target_term!(container, parent_param, parent_mult, target)
    end
    return
end

function update_container_parameter_values!(
    optimization_container::OptimizationContainer,
    model::OperationModel,
    key::ParameterKey{T, U},
    input::DatasetContainer{InMemoryDataset},
) where {T <: TimeSeriesCostParameter, U <: PSY.Component}
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
    _apply_cost_targets!(optimization_container, param_container)
    return
end

function update_container_parameter_values!(
    optimization_container::OptimizationContainer,
    model::OperationModel,
    key::ParameterKey{T, U},
    input::DatasetContainer{InMemoryDataset},
) where {T <: TimeSeriesCostParameter, U <: PSY.Service}
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
    _apply_cost_targets!(optimization_container, param_container)
    return
end
