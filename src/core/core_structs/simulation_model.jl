mutable struct SimulationRef
    raw::String
    models::String
    results::String
    run_count::Dict{Int64, Dict{Int64, Int64}}
    date_ref::Dict{Int64, Dates.DateTime}
    current_time::Dates.DateTime
    reset::Bool
end

function _initialize_sim_ref(steps::Int64, stages::Base.KeySet)

    count_dict = Dict{Int64, Dict{Int64, Int64}}()

    for s in 1:steps
        count_dict[s] = Dict{Int64, Int64}()
        for st in stages
            count_dict[s][st] = 0
        end
    end

    return sim_ref = SimulationRef("init",
                                   "init",
                                   "init",
                                   count_dict,
                                   Dict{Int64, Dates.DateTime}(),
                                   Dates.now(),
                                   true
                                   )

end

mutable struct Stage
    key::Int64
    model::OperationModel
    execution_count::Int64
    solver::String

    function Stage(key::Int64,
                   model::OperationModel,
                   execution_count::Int64)

    new(key,
        model,
        execution_count,
        JuMP.solver_name(model.canonical.JuMPmodel)
        )

    end

end

function set_stage_optimizer!(stage::Stage, optimizer_factory::JuMP.OptimizerFactory)
    JuMP.set_optimizer(stage.model.canonical_mode.JuMPmodel,
                       optimizer_factory)
    stage.solver = JuMP.solver_name(stage.model.canonical_mode.JuMPmodel)
end

mutable struct Simulation
    steps::Int64
    stages::Vector{Stage}
    valid_timeseries::Bool
    daterange::NTuple{2, Dates.DateTime} #Inital Time of the first forecast and Inital Time of the last forecast
    ref::SimulationRef


    function Simulation(base_name::String,
                        steps::Int64,
                        stages::Dict{Int64, Tuple{ModelReference{T}, PSY.System, Int64, JuMP.OptimizerFactory}},
                        simulation_folder::String;
                        kwargs...) where {T<:PM.AbstractPowerFormulation}


    sim_ref = _initialize_sim_ref(steps, keys(stages))

    dates, validation, stages_vector = build_simulation!(sim_ref,
                                                        base_name,
                                                        steps,
                                                        stages,
                                                        simulation_folder;
                                                        kwargs...)

    new(steps,
        stages_vector,
        validation,
        dates,
        sim_ref)

    end

end

get_steps(s::Simulation) = s.steps
