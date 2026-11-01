"""
    add_constraint_coefficients!(container, C, V, P, components, model;
        constraint_meta = ISOPT.CONTAINER_KEY_EMPTY_META,
        variable_meta = ISOPT.CONTAINER_KEY_EMPTY_META,
        parameter_meta = ISOPT.CONTAINER_KEY_EMPTY_META, scale = 1.0)

Bind matching `(component, time)` cells of constraint type `C` and variable type `V`
to numeric parameter type `P <: ConstraintCoefficientParameter`. Immediately replace
each complete normalized coefficient with `Float64(scale) * value * multiplier` and
register it for automatic simulation updates. Returns `nothing`.

# Arguments

  - `container::OptimizationContainer`: container holding all three arrays.
  - `C`, `V`, `P`: constraint, variable and coefficient parameter types.
  - `components`: selected devices for a `DeviceModel`, or one service for a `ServiceModel`.
  - `model`: device or service model supplying the component type.
  - `constraint_meta`, `variable_meta`, `parameter_meta`: independent container metadata.
    For a service model, parameter metadata defaults to and must equal the service name.
  - `scale`: finite factor applied to every selected coefficient.

Call this inside `add_constraints!` during `ModelConstructStage`, after storing all
constraint rows. Argument-stage parameter creation is independent of binding. Parameter
rows follow the component-to-time-series mapping; multipliers remain component-specific.
The entire batch is validated before modification. Each constraint-variable pair permits
only one binding across the container. A zero coefficient removes the previous term.

Constraint and variable arrays must have matching component/time cells. Service arrays
must both use service/time cells; contributing-device axes require explicit
[`add_coefficient_target!`](@ref) calls. Only scalar affine constraints and dense,
two-dimensional Float64 parameter storage are supported. `rebuild_model = true` is rejected.
"""
function add_constraint_coefficients!(
    container::OptimizationContainer,
    ::Type{C},
    ::Type{V},
    ::Type{P},
    devices,
    model::DeviceModel;
    constraint_meta = ISOPT.CONTAINER_KEY_EMPTY_META,
    variable_meta = ISOPT.CONTAINER_KEY_EMPTY_META,
    parameter_meta = ISOPT.CONTAINER_KEY_EMPTY_META,
    scale = 1.0,
) where {C <: ConstraintType, V <: VariableType, P <: ConstraintCoefficientParameter}
    return _add_constraint_coefficients!(
        container, C, V, P, PSY.get_name.(collect(devices)), get_component_type(model),
        constraint_meta, variable_meta, parameter_meta, scale,
    )
end

function add_constraint_coefficients!(
    container::OptimizationContainer,
    ::Type{C},
    ::Type{V},
    ::Type{P},
    service::S,
    model::ServiceModel{S};
    constraint_meta = ISOPT.CONTAINER_KEY_EMPTY_META,
    variable_meta = ISOPT.CONTAINER_KEY_EMPTY_META,
    parameter_meta = get_service_name(model),
    scale = 1.0,
) where {
    C <: ConstraintType, V <: VariableType, P <: ConstraintCoefficientParameter,
    S <: PSY.Service,
}
    name = PSY.get_name(service)
    if name != get_service_name(model) || parameter_meta != name
        throw(
            ArgumentError(
                "Parameter $P requires service-name metadata '$name' and a matching ServiceModel",
            ),
        )
    end
    return _add_constraint_coefficients!(
        container, C, V, P, [name], S, constraint_meta, variable_meta, parameter_meta, scale,
    )
end

function _add_constraint_coefficients!(
    container, ::Type{C}, ::Type{V}, ::Type{P}, names, component_type,
    constraint_meta, variable_meta, parameter_meta, scale,
) where {C <: ConstraintType, V <: VariableType, P <: ConstraintCoefficientParameter}
    key = ParameterKey(P, component_type, parameter_meta)
    _check_coefficient_build_mode(container, key)
    constraints =
        get_constraint(container, ConstraintKey(C, component_type, constraint_meta))
    variables = get_variable(container, VariableKey(V, component_type, variable_meta))
    param = get_parameter(container, key)
    values, multipliers = _coefficient_storage(param, key)
    factor = _coefficient_scale(scale)
    targets = CoefficientTarget[]
    time_steps = collect(get_time_steps(container))
    for name in names, t in time_steps
        constraint = _coefficient_cell(constraints, name, t, "constraint $C")
        variable = _coefficient_cell(variables, name, t, "variable $V")
        push!(
            targets,
            _prepare_coefficient_target(param, key, name, t,
                constraint, variable, factor, values, multipliers),
        )
    end
    _register_coefficient_targets!(container, param, key, targets)
    return
end

"""
    add_coefficient_target!(container, key, name, t, constraint, variable, scale)

Bind one complete normalized coefficient to `scale * value * multiplier` from a
[`ConstraintCoefficientParameter`](@ref). Apply it immediately and register future
updates. Returns `nothing`; use this for mappings with nonmatching target axes.

# Arguments

  - `container::OptimizationContainer`: container holding the parameter and JuMP model.
  - `key::ParameterKey`: coefficient parameter key, including its metadata.
  - `name::String`: component name used for the time-series mapping and multiplier row.
  - `t::Int`: parameter time step.
  - `constraint::JuMP.ConstraintRef`: scalar affine constraint to modify.
  - `variable::JuMP.VariableRef`: variable in the same JuMP model.
  - `scale::Real`: finite coefficient scale.

Each constraint-variable pair may have only one binding across all parameter keys.
The parameter storage must be dense and two-dimensional. `rebuild_model = true` is
unsupported. See [`add_constraint_coefficients!`](@ref) for matching-axis batch binding.
"""
function add_coefficient_target!(
    container::OptimizationContainer,
    key::ParameterKey{P, U},
    name::String,
    t::Int,
    constraint::JuMP.ConstraintRef,
    variable::JuMP.VariableRef,
    scale::Real,
) where {P <: ConstraintCoefficientParameter, U <: PSY.Component}
    _check_coefficient_build_mode(container, key)
    param = get_parameter(container, key)
    values, multipliers = _coefficient_storage(param, key)
    target = _prepare_coefficient_target(param, key, name, t, constraint, variable,
        _coefficient_scale(scale), values, multipliers)
    _register_coefficient_targets!(container, param, key, [target])
    return
end

function add_coefficient_target!(
    container::OptimizationContainer,
    ::P,
    ::Type{U},
    name::String,
    t::Int,
    constraint::JuMP.ConstraintRef,
    variable::JuMP.VariableRef,
    scale::Real;
    meta = ISOPT.CONTAINER_KEY_EMPTY_META,
) where {P <: ConstraintCoefficientParameter, U <: PSY.Component}
    return add_coefficient_target!(container, ParameterKey(P, U, meta), name, t,
        constraint, variable, scale)
end

function _check_coefficient_build_mode(container, key)
    if get_rebuild_model(get_settings(container))
        throw(ArgumentError("$key does not support models built with rebuild_model = true"))
    end
    return
end

function _coefficient_scale(scale)
    factor = Float64(scale)
    isfinite(factor) || throw(ArgumentError("Coefficient scale must be finite, got $scale"))
    return factor
end

function _coefficient_storage(param::ParameterContainer, key)
    return _coefficient_storage(
        get_parameter_array(param),
        get_multiplier_array(param),
        key,
    )
end

function _coefficient_storage(values, multipliers, key)
    throw(
        ArgumentError(
            "$key requires dense, two-dimensional Float64 parameter and multiplier storage",
        ),
    )
end

function _coefficient_storage(
    values::DenseAxisArray{Float64, 2}, multipliers::DenseAxisArray{Float64, 2}, key,
)
    if axes(values, 2) != 1:size(values, 2) || axes(multipliers, 2) != axes(values, 2)
        throw(
            ArgumentError(
                "$key requires matching parameter/multiplier time axes numbered 1:n",
            ),
        )
    end
    return values, multipliers
end

function _coefficient_cell(array, name, t, description)
    throw(ArgumentError("$description requires two-dimensional (component, time) indexing"))
end

function _coefficient_cell(array::AbstractArray{T, 2}, name, t, description) where {T}
    try
        return array[name, t]
    catch err
        throw(
            ArgumentError(
                "Missing $description cell for component '$name' at time $t: $(sprint(showerror, err))",
            ),
        )
    end
end

function _prepare_coefficient_target(
    param, key, name, t, constraint, variable, scale, values, multipliers,
)
    attributes = get_attributes(param)
    if !(name in get_component_names(attributes))
        throw(ArgumentError("Component '$name' has no required coefficient parameter $key"))
    end
    uuid = _get_ts_uuid(attributes, name)
    _coefficient_cell(values, uuid, t, "parameter $key for component '$name'")
    _coefficient_cell(multipliers, name, t, "multiplier for $key")
    return CoefficientTarget(constraint, variable, scale,
        values.lookup[1][uuid], multipliers.lookup[1][name], t)
end

function _collect_coefficient_pairs!(pairs, ::ParameterAttributes)
    return
end

function _collect_coefficient_pairs!(pairs, attributes::TimeSeriesAttributes)
    for target in get_coefficient_targets(attributes)
        push!(pairs, (target.constraint, target.variable))
    end
    return
end

function _check_linear_coefficient_constraint(constraint)
    throw(ArgumentError("Coefficient binding requires a scalar linear constraint"))
end

function _check_linear_coefficient_constraint(
    ::JuMP.ScalarConstraint{JuMP.AffExpr, S},
) where {S}
    return
end

function _validate_coefficient_target(model, target, pairs, key)
    constraint, variable = target.constraint, target.variable
    if JuMP.owner_model(constraint) !== model || JuMP.owner_model(variable) !== model
        throw(
            ArgumentError(
                "Coefficient target for $key must belong to the container's JuMP model",
            ),
        )
    end
    if !JuMP.is_valid(model, constraint) || !JuMP.is_valid(model, variable)
        throw(
            ArgumentError(
                "Coefficient target for $key must use valid constraint and variable references",
            ),
        )
    end
    _check_linear_coefficient_constraint(JuMP.constraint_object(constraint))
    isfinite(target.scale) ||
        throw(ArgumentError("Coefficient scale for $key must be finite"))
    pair = (constraint, variable)
    if pair in pairs
        throw(
            ArgumentError(
                "Coefficient of $variable in $constraint is already bound (parameter $key)",
            ),
        )
    end
    push!(pairs, pair)
    return
end

function _register_coefficient_targets!(container, param, key, targets)
    pairs = Set{Tuple{JuMP.ConstraintRef, JuMP.VariableRef}}()
    for stored_param in values(get_parameters(container))
        _collect_coefficient_pairs!(pairs, get_attributes(stored_param))
    end
    coefficients = Float64[]
    parent_param = get_parameter_array_data(param)
    parent_mult = get_multiplier_array_data(param)
    for target in targets
        _validate_coefficient_target(get_jump_model(container), target, pairs, key)
        push!(coefficients, _coefficient_value(parent_param, parent_mult, target))
    end
    registered = get_coefficient_targets(get_attributes(param))
    for (target, coefficient) in zip(targets, coefficients)
        _set_coefficient_target!(target, coefficient)
        push!(registered, target)
    end
    return
end

function _coefficient_value(parent_param, parent_mult, target::CoefficientTarget)
    value =
        target.scale * parent_param[target.param_row, target.time_step] *
        parent_mult[target.multiplier_row, target.time_step]
    if !isfinite(value)
        throw(
            ArgumentError(
                "Effective coefficient of $(target.variable) in $(target.constraint) at time $(target.time_step) must be finite, got $value",
            ),
        )
    end
    return value
end

function _set_coefficient_target!(target::CoefficientTarget, value::Float64)
    try
        JuMP.set_normalized_coefficient(target.constraint, target.variable, value)
    catch err
        error(
            "Failed to apply coefficient of $(target.variable) in $(target.constraint) at time $(target.time_step): $(sprint(showerror, err))",
        )
    end
    return
end

function _apply_coefficient_target!(
    parent_param::Matrix{Float64}, parent_mult::Matrix{Float64}, target::CoefficientTarget,
)
    _set_coefficient_target!(target, _coefficient_value(parent_param, parent_mult, target))
    return
end

function _apply_coefficient_targets!(param_container::ParameterContainer)
    parent_param = get_parameter_array_data(param_container)
    parent_mult = get_multiplier_array_data(param_container)
    targets = get_coefficient_targets(get_attributes(param_container))
    coefficients =
        [_coefficient_value(parent_param, parent_mult, target) for target in targets]
    for (target, coefficient) in zip(targets, coefficients)
        _set_coefficient_target!(target, coefficient)
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
