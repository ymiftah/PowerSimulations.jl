"""
Cache for all model results
"""
mutable struct OptimizationOutputCaches
    data::Dict{OptimizationResultCacheKey, OptimizationOutputCache}
    "Sum of `get_size` over all entries of `data`. Every mutation of a per-key cache
    through this module updates it."
    size::Int
    max_size::Int
    min_flush_size::Int
end

function OptimizationOutputCaches()
    return OptimizationOutputCaches(
        Dict{OptimizationResultCacheKey, OptimizationOutputCache}(),
        0,
        0,
        0,
    )
end

function OptimizationOutputCaches(rules::CacheFlushRules)
    return OptimizationOutputCaches(
        Dict{OptimizationResultCacheKey, OptimizationOutputCache}(),
        0,
        rules.max_size,
        rules.min_flush_size,
    )
end

"""
    Base.empty!(cache::OptimizationOutputCaches)

Empty the [`OptimizationOutputCaches`](@ref)
"""
function Base.empty!(cache::OptimizationOutputCaches)
    for output_cache in values(cache.data)
        empty!(output_cache)
    end
    cache.size = 0
    return
end

get_max_size(cache::OptimizationOutputCaches) = cache.max_size
get_min_flush_size(cache::OptimizationOutputCaches) = cache.min_flush_size
get_size(cache::OptimizationOutputCaches) = cache.size

# Full recomputation of the tracked size, used to check the incremental total.
_compute_size(cache::OptimizationOutputCaches) =
    reduce(+, (get_size(x) for x in values(cache.data)); init = 0)

# Leave some buffer because we may slightly exceed the limit.
is_full(cache::OptimizationOutputCaches, cur_size) = cur_size >= cache.max_size * 0.95

function add_output_cache!(cache::OptimizationOutputCaches, model_name, key, flush_rule)
    cache_key = OptimizationResultCacheKey(model_name, key)
    _set_output_cache!(
        cache,
        cache_key,
        OptimizationOutputCache(cache_key, flush_rule),
    )
    @debug "Added cache container for" LOG_GROUP_SIMULATION_STORE model_name key flush_rule
    return
end

# Insert or replace the per-key cache under `cache_key`, keeping the tracked size in sync.
function _set_output_cache!(
    cache::OptimizationOutputCaches,
    cache_key::OptimizationResultCacheKey,
    output_cache::OptimizationOutputCache,
)
    if haskey(cache.data, cache_key)
        cache.size -= get_size(cache.data[cache_key])
    end
    cache.data[cache_key] = output_cache
    cache.size += get_size(output_cache)
    return
end

"""
Add a result to `output_cache`, a per-key cache stored in `cache`, and update the tracked
size.
"""
function add_result!(
    cache::OptimizationOutputCaches,
    output_cache::OptimizationOutputCache,
    timestamp::Dates.DateTime,
    array::Array{Float64},
    system_cache_is_full::Bool,
)
    cache.size += add_result!(output_cache, timestamp, array, system_cache_is_full)
    return
end

"""
Discard `timestamps` from `output_cache`, a per-key cache stored in `cache`, and update the
tracked size.
"""
function discard_results!(
    cache::OptimizationOutputCaches,
    output_cache::OptimizationOutputCache,
    timestamps,
)
    cache.size += discard_results!(output_cache, timestamps)
    return
end

"""
Return true if the cache has data that has not been flushed to storage.
"""
function has_dirty(cache::OptimizationOutputCaches)
    for output_cache in values(cache.data)
        if has_dirty(output_cache)
            return true
        end
    end

    return false
end

get_output_cache(cache::OptimizationOutputCaches, key::OptimizationResultCacheKey) =
    cache.data[key]

function get_output_cache(
    cache::OptimizationOutputCaches,
    model_name,
    key::OptimizationContainerKey,
)
    cache_key = OptimizationResultCacheKey(model_name, key)
    return get_output_cache(cache, cache_key)
end

"""
Return true if the data for `timestamp` is stored in cache.
"""
function is_cached(cache::OptimizationOutputCaches, model_name, key, index)
    cache_key = OptimizationResultCacheKey(model_name, key)
    return is_cached(cache, cache_key, index)
end

is_cached(cache::OptimizationOutputCaches, key, timestamp::Dates.DateTime) =
    has_timestamp(cache.data[key], timestamp::Dates.DateTime)

is_cached(cache::OptimizationOutputCaches, key, ::Int) = false

"""
Log the cache hit percentages for all caches.
"""
function log_cache_hit_percentages(cache::OptimizationOutputCaches)
    for key in keys(cache.data)
        output_cache = cache.data[key]
        cache_hit_pecentage = get_cache_hit_percentage(output_cache)
        @debug "Cache stats" LOG_GROUP_SIMULATION_STORE key cache_hit_pecentage
    end
    return
end

"""
Read the result from cache. Callers must first call [`is_cached`](@ref) to check if the
timestamp is present.
"""
function read_result(cache::OptimizationOutputCaches, model_name, key, timestamp)
    cache_key = OptimizationResultCacheKey(model_name, key)
    return read_result(cache, cache_key, timestamp)
end

read_result(cache::OptimizationOutputCaches, key, timestamp) =
    cache.data[key].data[timestamp]
