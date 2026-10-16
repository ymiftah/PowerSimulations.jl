# Tests for the memoised `is_milp` and the once-per-container MILP dual warning.
struct IsMILPTestConstraint <: PSI.ConstraintType end

@testset "is_milp tracks binary variables and memoises the result" begin
    sys = PSB.build_system(PSITestSystems, "c_sys5")
    container =
        PSI.OptimizationContainer(sys, PSI.Settings(sys), nothing, PSY.SingleTimeSeries)
    jm = PSI.get_jump_model(container)

    @test !PSI.is_milp(container)
    @test !PSI.is_milp(container)
    @test container.milp_state.zero_one_count == 0

    continuous = PSI.add_variable_container!(
        container, PSI.ActivePowerVariable(), PSY.ThermalStandard, 1:2)
    continuous[1] = JuMP.@variable(jm, base_name = "p1")
    continuous[2] = JuMP.@variable(jm, base_name = "p2")
    @test !PSI.is_milp(container)
    @test container.milp_state.zero_one_count == 0

    binary = PSI.add_variable_container!(
        container, PSI.OnVariable(), PSY.ThermalStandard, 1:2)
    binary[1] = JuMP.@variable(jm, binary = true, base_name = "on1")
    binary[2] = JuMP.@variable(jm, binary = true, base_name = "on2")
    @test PSI.is_milp(container)
    @test container.milp_state.zero_one_count == 2
    for _ in 1:1000
        @test PSI.is_milp(container)
    end

    # Unsetting the binaries (as process_duals does) must invalidate the memo.
    JuMP.unset_binary.(binary)
    @test !PSI.is_milp(container)
    JuMP.set_binary.(binary)
    @test PSI.is_milp(container)
end

@testset "MILP dual warning is emitted at most once per container" begin
    sys = PSB.build_system(PSITestSystems, "c_sys5")
    container =
        PSI.OptimizationContainer(sys, PSI.Settings(sys), nothing, PSY.SingleTimeSeries)
    jm = PSI.get_jump_model(container)

    binary = PSI.add_variable_container!(
        container, PSI.OnVariable(), PSY.ThermalStandard, 1:2)
    binary[1] = JuMP.@variable(jm, binary = true, base_name = "on1")
    binary[2] = JuMP.@variable(jm, binary = true, base_name = "on2")

    @test_logs (:warn, r"MILP") PSI.add_dual_container!(
        container, IsMILPTestConstraint, PSY.ThermalStandard, 1:2; meta = "a")
    @test_logs PSI.add_dual_container!(
        container, IsMILPTestConstraint, PSY.ThermalStandard, 1:2; meta = "b")
    @test_logs PSI.add_dual_container!(
        container, IsMILPTestConstraint, PSY.ThermalStandard, 1:2; meta = "c")
end
