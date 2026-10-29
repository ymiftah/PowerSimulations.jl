function get_time_series_values!(
    time_series_type::Type{T},
    model::DecisionModel,
    component,
    name::String,
    initial_time::Dates.DateTime,
    horizon::Int;
    ignore_scaling_factors = true,
    interval::Dates.Millisecond = UNSET_INTERVAL,
) where {T <: PSY.Forecast}
    is_interval = _to_is_interval(interval)
    settings = get_settings(model)
    resolution = get_resolution(settings)
    key = IS.TimeSeriesCacheKey(IS.get_uuid(component), T, name, resolution, is_interval)
    if !use_time_series_cache(settings)
        metadata = _get_time_series_metadata!(
            model,
            key,
            T,
            component,
            name;
            interval = is_interval,
        )
        return _read_time_series_values(
            T,
            component,
            metadata,
            initial_time,
            horizon,
            ignore_scaling_factors,
        )
    end

    cache = get_time_series_cache(model)
    if haskey(cache, key)
        ts_cache = cache[key]
    else
        ts_cache = IS.make_time_series_cache(
            time_series_type,
            component,
            name,
            initial_time,
            horizon;
            ignore_scaling_factors = ignore_scaling_factors,
            interval = is_interval,
            resolution = resolution,
        )
        cache[key] = ts_cache
    end

    ts = IS.get_time_series_array!(ts_cache, initial_time)
    return TimeSeries.values(ts)
end

function get_time_series_values!(
    ::Type{T},
    model::EmulationModel,
    component::U,
    name::String,
    initial_time::Dates.DateTime,
    len::Int = 1;
    ignore_scaling_factors = true,
    resolution::Dates.Millisecond = UNSET_RESOLUTION,
) where {T <: PSY.StaticTimeSeries, U <: PSY.Component}
    settings = get_settings(model)
    key_resolution =
        resolution == UNSET_RESOLUTION ? get_resolution(settings) : resolution
    is_resolution = _to_is_resolution(key_resolution)
    key = IS.TimeSeriesCacheKey(IS.get_uuid(component), T, name, key_resolution, nothing)
    if !use_time_series_cache(settings)
        metadata = _get_time_series_metadata!(
            model,
            key,
            T,
            component,
            name;
            resolution = is_resolution,
        )
        return _read_time_series_values(
            T,
            component,
            metadata,
            initial_time,
            len,
            ignore_scaling_factors,
        )
    end

    cache = get_time_series_cache(model)
    if haskey(cache, key)
        ts_cache = cache[key]
    else
        ts_cache = IS.make_time_series_cache(
            T,
            component,
            name,
            initial_time,
            len;
            ignore_scaling_factors = ignore_scaling_factors,
            resolution = is_resolution,
        )
        cache[key] = ts_cache
    end

    ts = IS.get_time_series_array!(ts_cache, initial_time)
    return TimeSeries.values(ts)
end

# Metadata is resolved once per key and model build; the system's time series do not
# change while a model is built.
function _get_time_series_metadata!(
    model::OperationModel,
    key::IS.TimeSeriesCacheKey,
    ::Type{T},
    component,
    name::String;
    kwargs...,
) where {T <: PSY.TimeSeriesData}
    cache = get_time_series_metadata_cache(get_optimization_container(model))
    return get!(cache, key) do
        IS.get_time_series_metadata(T, component, name; kwargs...)
    end
end

# Same read as `IS.get_time_series_values(T, component, name; start_time, len, ...)` for
# one window, starting from resolved metadata instead of a metadata-store query.
function _read_time_series_values(
    ::Type{T},
    component,
    metadata::IS.TimeSeriesMetadata,
    start_time::Dates.DateTime,
    len::Int,
    ignore_scaling_factors::Bool,
) where {T <: PSY.TimeSeriesData}
    start_time = IS._check_start_time(start_time, metadata)
    rows = IS._get_rows(start_time, len, metadata)
    columns = IS._get_columns(start_time, 1, metadata)
    storage = IS.get_time_series_storage(component)
    ts = IS.deserialize_time_series(T, storage, metadata, rows, columns)
    return IS.get_time_series_values(
        component,
        ts;
        start_time = start_time,
        len = len,
        ignore_scaling_factors = ignore_scaling_factors,
    )
end
