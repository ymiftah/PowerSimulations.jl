# A failed dual LP must restore the integer variables and make the solve report FAILED.
@testset "Failed dual LP restores integer variables and reports FAILED" begin
    sys = PSB.build_system(PSITestSystems, "c_sys5_uc")
    template = ProblemTemplate(
        NetworkModel(CopperPlatePowerModel; duals = [CopperPlateBalanceConstraint]),
    )
    set_device_model!(template, ThermalStandard, ThermalBasicUnitCommitment)
    set_device_model!(template, PowerLoad, StaticPowerLoad)
    model = DecisionModel(
        template,
        sys;
        name = "UC",
        optimizer = HiGHS_optimizer,
        optimizer_solve_log_print = false,
    )
    @test build!(model; output_dir = mktempdir()) == PSI.ModelBuildStatus.BUILT
    solve!(model)
    @test PSI.get_run_status(model) == PSI.RunStatus.SUCCESSFULLY_FINALIZED

    container = PSI.get_optimization_container(model)
    jm = PSI.get_jump_model(container)
    # The dual LP restore leaves the model modified; re-solve to read values.
    JuMP.optimize!(jm)
    binary = first(v for v in JuMP.all_variables(jm) if JuMP.is_binary(v))
    lb_before = JuMP.has_lower_bound(binary) ? JuMP.lower_bound(binary) : nothing
    ub_before = JuMP.has_upper_bound(binary) ? JuMP.upper_bound(binary) : nothing
    zero_one_before = JuMP.num_constraints(jm, JuMP.VariableRef, MOI.ZeroOne)
    @test zero_one_before > 0

    # A zero time limit leaves HiGHS without a feasible primal point, so the dual LP fails.
    no_solution_optimizer = JuMP.optimizer_with_attributes(
        HiGHS.Optimizer,
        "time_limit" => 0.0,
        "presolve" => "off",
    )
    @test PSI.process_duals(container, no_solution_optimizer) == PSI.RunStatus.FAILED
    @test JuMP.primal_status(jm) ∉
          (MOI.FEASIBLE_POINT, MOI.NEARLY_FEASIBLE_POINT)

    @test JuMP.num_constraints(jm, JuMP.VariableRef, MOI.ZeroOne) == zero_one_before
    @test JuMP.is_binary(binary)
    @test !JuMP.is_fixed(binary)
    @test (JuMP.has_lower_bound(binary) ? JuMP.lower_bound(binary) : nothing) == lb_before
    @test (JuMP.has_upper_bound(binary) ? JuMP.upper_bound(binary) : nothing) == ub_before

    # The dual LP of calculate_dual_variables! uses the container's optimizer.
    original_settings = container.settings
    container.settings = PSI.Settings(sys; optimizer = no_solution_optimizer)
    JuMP.set_optimizer(jm, HiGHS_optimizer)
    JuMP.optimize!(jm)
    @test PSI.calculate_dual_variables!(container, sys, true) == PSI.RunStatus.FAILED
    @test JuMP.is_binary(binary)
    @test !JuMP.is_fixed(binary)

    # The same failure propagates through solve_impl!.
    JuMP.set_optimizer(jm, HiGHS_optimizer)
    @test PSI.solve_impl!(container, sys) == PSI.RunStatus.FAILED
    container.settings = original_settings
    @test JuMP.is_binary(binary)
    @test !JuMP.is_fixed(binary)
end

# Primal start values are cleared for the dual LP and restored afterwards.
@testset "Dual LP clears and restores primal start values" begin
    sys = PSB.build_system(PSITestSystems, "c_sys5_uc")
    template = ProblemTemplate(
        NetworkModel(CopperPlatePowerModel; duals = [CopperPlateBalanceConstraint]),
    )
    set_device_model!(template, ThermalStandard, ThermalBasicUnitCommitment)
    set_device_model!(template, PowerLoad, StaticPowerLoad)
    model = DecisionModel(
        template,
        sys;
        name = "UC",
        optimizer = HiGHS_optimizer,
        optimizer_solve_log_print = false,
    )
    @test build!(model; output_dir = mktempdir()) == PSI.ModelBuildStatus.BUILT
    solve!(model)
    container = PSI.get_optimization_container(model)
    jm = PSI.get_jump_model(container)

    binaries = [v for v in JuMP.all_variables(jm) if JuMP.is_binary(v)]
    continuous = first(
        v for v in JuMP.all_variables(jm) if !JuMP.is_binary(v) && !JuMP.is_integer(v)
    )
    for (i, b) in enumerate(binaries[1:min(3, length(binaries))])
        JuMP.set_start_value(b, isodd(i) ? 1.0 : 0.0)
    end
    JuMP.set_start_value(continuous, 0.5)
    starts_before = Dict(v => JuMP.start_value(v) for v in JuMP.all_variables(jm))
    # Setting starts drops the previous solution, so re-solve with them before the dual LP.
    JuMP.optimize!(jm)

    @test PSI.process_duals(container, HiGHS_optimizer) ==
          PSI.RunStatus.SUCCESSFULLY_FINALIZED
    @test all(JuMP.start_value(v) == start for (v, start) in starts_before)
end
