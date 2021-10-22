"""
_calculate_interval_inner_counts(intervals::OrderedDict{String,<:Dates.TimePeriod})

Calculates how many times a problem is executed for every interval of the previous problem
"""
function _calculate_interval_inner_counts(intervals::OrderedDict{Symbol, Dates.Period})
    order = collect(keys(intervals))
    reverse_order = length(intervals):-1:1
    interval_run_counts = Dict{Int, Int}(1 => 1)
    for k in reverse_order[1:(end - 1)]
        problem_name = order[k]
        previous_problem_name = order[k - 1]
        problem_interval = intervals[problem_name]
        previous_problem_interval = intervals[previous_problem_name]
        if Dates.Millisecond(previous_problem_interval % problem_interval) !=
           Dates.Millisecond(0)
            throw(
                IS.ConflictingInputsError(
                    "The interval configuration provided results in a fractional number of executions of problem $problem_name",
                ),
            )
        end
        interval_run_counts[k] = previous_problem_interval / problem_interval
        @debug "problem $k is executed $(interval_run_counts[k]) time within each interval of problem $(k-1)"
    end
    return interval_run_counts
end

""" Function calculates the total number of problem executions in the simulation and allocates the appropiate vector"""
function _allocate_execution_order(interval_run_counts::Dict{Int, Int})
    total_size_of_vector = 0
    for (k, counts) in interval_run_counts
        mult = 1
        for i in 1:k
            mult *= interval_run_counts[i]
        end
        total_size_of_vector += mult
    end
    return -1 * ones(Int, total_size_of_vector)
end

function _fill_execution_order(
    execution_order::Vector{Int},
    interval_run_counts::Dict{Int, Int},
)
    function _fill_problem(index::Int, problem::Int)
        if problem < last_problem
            next_problem = problem + 1
            for i in 1:interval_run_counts[next_problem]
                index = _fill_problem(index, next_problem)
            end
        end
        execution_order[index] = problem
        index -= 1
    end

    index = length(execution_order)
    problems = sort!(collect(keys(interval_run_counts)))
    last_problem = problems[end]
    _fill_problem(index, problems[1])
    return
end

function _get_execution_order_vector(intervals::OrderedDict{Symbol, Dates.Period})
    length(intervals) == 1 && return [1]
    interval_run_counts = _calculate_interval_inner_counts(intervals)
    execution_order_vector = _allocate_execution_order(interval_run_counts)
    _fill_execution_order(execution_order_vector, interval_run_counts)
    @assert isempty(findall(x -> x == -1, execution_order_vector))
    return execution_order_vector
end

function _get_num_executions_by_problem(
    models::SimulationModels,
    execution_order::Vector{Int},
)
    model_names = get_decision_model_names(models)
    executions_by_problem = Dict(x => 0 for x in model_names)
    for number in execution_order
        executions_by_problem[model_names[number]] += 1
    end
    return executions_by_problem
end

@doc raw"""
    SimulationSequence(
                        models::SimulationModels,
                        feedforward::Dict{Symbol, <:AbstractAffectFeedforward}
                        ini_cond_chronology::Dict{Symbol, <:FeedforwardChronology}
                        cache::Dict{Symbol, AbstractCache}
                        )
"""
mutable struct SimulationSequence
    horizons::OrderedDict{Symbol, Int}
    intervals::OrderedDict{Symbol, Dates.Period}
    feedforwards::Dict{<:Tuple, <:AbstractAffectFeedforward}
    ini_cond_chronology::InitialConditionChronology
    execution_order::Vector{Int}
    executions_by_problem::Dict{Symbol, Int}
    current_execution_index::Int64
    uuid::Base.UUID

    function SimulationSequence(;
        models::SimulationModels,
        feedforward = Dict{Symbol, AbstractAffectFeedforward}(),
        ini_cond_chronology = InterProblemChronology(),
    )
        # Allow strings or symbols as keys; convert to symbols.
        intervals = PSI.determine_intervals(models)
        horizons = PSI.determine_horizons!(models)
        step_resolution = first(intervals)[2]
        if length(models.decision_models) == 1
            ini_cond_chronology = IntraProblemChronology()
        end
        execution_order = PSI._get_execution_order_vector(intervals)
        executions_by_problem = PSI._get_num_executions_by_problem(models, execution_order)
        sequence_uuid = IS.make_uuid()
        initialize_simulation_internals!(models, sequence_uuid)
        new(
            horizons,
            step_resolution,
            intervals,
            feedforward,
            ini_cond_chronology,
            execution_order,
            executions_by_problem,
            0,
            sequence_uuid,
        )
    end
end

get_step_resolution(sequence::SimulationSequence) = sequence.step_resolution

function get_problem_interval_chronology(sequence::SimulationSequence, problem)
    return sequence.intervals[problem][2]
end

function get_interval(sequence::SimulationSequence, problem::Symbol)
    return sequence.intervals[problem][1]
end

function get_interval(sequence::SimulationSequence, model)
    return sequence.intervals[model][1]
end

get_execution_order(sequence::SimulationSequence) = sequence.execution_order
