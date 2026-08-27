"""
    add_constraints!(container, ::Type{CopperPlateBalanceConstraint}, sys, model::NetworkModel{T})

Enforces the network's active power balance over whichever aggregate the formulation declares
through [`balance_aggregation`](@ref): `PSY.System` for `CopperPlatePowerModel` and
`PTDFPowerModel` (one row per synchronous subnetwork), `PSY.Area` for `AreaBalancePowerModel`
and `AreaPTDFPowerModel` (one row per area).

The constraint's component-type key and its row axis both come from the balance expression, so
it is keyed identically to that expression and to the dual `assign_dual_variable!` registers.
"""
function add_constraints!(
    container::OptimizationContainer,
    ::Type{CopperPlateBalanceConstraint},
    ::PSY.System,
    ::NetworkModel{T},
) where {T <: PM.AbstractPowerModel}
    aggregation = balance_aggregation(T)
    expressions = get_expression(container, ActivePowerBalance(), aggregation)
    aggregate_names, time_steps = axes(expressions)
    constraint = add_constraints_container!(
        container,
        CopperPlateBalanceConstraint(),
        aggregation,
        aggregate_names,
        time_steps,
    )
    jm = get_jump_model(container)
    for t in time_steps, k in aggregate_names
        constraint[k, t] = JuMP.@constraint(jm, expressions[k, t] == 0)
    end

    return
end
