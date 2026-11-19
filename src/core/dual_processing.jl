struct VarRestoreInfo{A <: AbstractArray}
    lb::Union{Nothing, A}
    ub::Union{Nothing, A}
    fixed_int_value::Union{Nothing, A}
    is_integer::Bool
end

_first_element(v::DenseAxisArray) = first(v)
_first_element(v::SparseAxisArray) = first(values(v.data))

function _round_cache_values!(cache::DenseAxisArray)
    cache.data .= round.(cache.data)
    return
end

function _round_cache_values!(cache::SparseAxisArray)
    for k in keys(cache.data)
        cache.data[k] = round(cache.data[k])
    end
    return
end

# DenseAxisArray duals broadcast over the backing array. Post-contingency
# duals are SparseAxisArray (Dict-backed), where `.data .= …` is undefined, so
# copy per key instead.
function _copy_dual_values!(dual::DenseAxisArray, constraint::DenseAxisArray)
    dual.data .= jump_value.(constraint).data
    return
end

function _copy_dual_values!(dual::SparseAxisArray, constraint::SparseAxisArray)
    for (k, cref) in constraint.data
        dual.data[k] = jump_value(cref)
    end
    return
end

# Restores the integer variables unset or fixed by `process_duals`.
function _restore_integer_variables!(var_container, cache)
    for (key, variable) in var_container
        if !haskey(cache, key)
            continue
        end
        info = cache[key]
        JuMP.unfix.(variable)
        if !isnothing(info.lb)
            JuMP.set_lower_bound.(variable, info.lb)
        end
        if !isnothing(info.ub)
            JuMP.set_upper_bound.(variable, info.ub)
        end
        if info.is_integer
            JuMP.set_integer.(variable)
        else
            JuMP.set_binary.(variable)
        end
        if !isnothing(info.fixed_int_value)
            JuMP.fix.(variable, info.fixed_int_value)
        end
    end
    return
end

# Clears the primal start values and returns them for `_restore_start_values!`.
function _clear_start_values!(jump_model)
    saved = Tuple{JuMP.VariableRef, Float64}[]
    for v in JuMP.all_variables(jump_model)
        start = JuMP.start_value(v)
        isnothing(start) && continue
        push!(saved, (v, start))
        JuMP.set_start_value(v, nothing)
    end
    return saved
end

function _restore_start_values!(saved)
    for (v, start) in saved
        JuMP.set_start_value(v, start)
    end
    return
end

function process_duals(container::OptimizationContainer, lp_optimizer)
    var_container = get_variables(container)
    for (k, v) in var_container
        container.primal_values_cache.variables_cache[k] = jump_value.(v)
    end

    for (k, v) in get_expressions(container)
        container.primal_values_cache.expressions_cache[k] = jump_value.(v)
    end
    var_cache = container.primal_values_cache.variables_cache
    cache = sizehint!(Dict{VariableKey, VarRestoreInfo}(), length(var_container))
    # The loop below modifies integer variables, so restore them on every exit path.
    saved_starts = Tuple{JuMP.VariableRef, Float64}[]
    try
        for (key, variable) in var_container
            is_integer_flag = false
            elem = _first_element(variable)
            if JuMP.is_binary(elem)
                JuMP.unset_binary.(variable)
            elseif JuMP.is_integer(elem)
                JuMP.unset_integer.(variable)
                is_integer_flag = true
            else
                continue
            end
            lb = if JuMP.has_lower_bound(elem)
                JuMP.lower_bound.(variable)
            else
                nothing
            end
            ub = if JuMP.has_upper_bound(elem)
                JuMP.upper_bound.(variable)
            else
                nothing
            end
            fixed_int_value = if JuMP.is_fixed(elem) && is_integer_flag
                jump_value.(variable)
            else
                nothing
            end
            cache[key] =
                VarRestoreInfo{typeof(var_cache[key])}(
                    lb,
                    ub,
                    fixed_int_value,
                    is_integer_flag,
                )
            # Round cached values in-place to nearest integer to avoid infeasibilities
            # from MIP solver numerical tolerances (e.g. 0.9999997 instead of 1.0)
            _round_cache_values!(var_cache[key])
            JuMP.fix.(variable, var_cache[key]; force = true)
        end
        @assert !isempty(cache)
        jump_model = get_jump_model(container)

        if JuMP.mode(jump_model) != JuMP.DIRECT
            JuMP.set_optimizer(jump_model, lp_optimizer)
        else
            @debug("JuMP model set in direct mode during dual calculation")
        end

        # Warm starts make HiGHS skip presolve, which fails on badly scaled objectives.
        saved_starts = _clear_start_values!(jump_model)
        JuMP.optimize!(jump_model)

        model_status = JuMP.primal_status(jump_model)
        if model_status ∉ (
            MOI.FEASIBLE_POINT::MOI.ResultStatusCode,
            MOI.NEARLY_FEASIBLE_POINT::MOI.ResultStatusCode,
        )
            @error "Optimizer returned $model_status during dual calculation"
            return RunStatus.FAILED
        end

        if JuMP.has_duals(jump_model)
            for (key, dual) in get_duals(container)
                constraint = get_constraint(container, key)
                _copy_dual_values!(dual, constraint)
            end
        end
    finally
        _restore_integer_variables!(var_container, cache)
        _restore_start_values!(saved_starts)
    end
    return RunStatus.SUCCESSFULLY_FINALIZED
end
