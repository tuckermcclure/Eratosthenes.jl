# add! a whole structure to the log.
function add!(log::HDF5Logger.Log, slug::String, t::Real, data, n::Int)
    function add_structured!(log, slug, data, n)
        if isbits(data) || (isa(data, Array) && isbits(eltype(data)))
            HDF5Logger.add!(log, slug, data, n) # HDF5Loggeres only know scalars, vectors, and matrices.
        else
            for field in fieldnames(data)
                add_structured!(log, slug * "/" * string(field), getfield(data, field), n)
            end
        end
    end
    HDF5Logger.add!(log, slug * "time", t, n)
    add_structured!(log, slug * "data", data, n)
end

# log! a whole structure.
function log!(log::HDF5Logger.Log, slug::String, t::Real, data)
    function log_structured!(log, slug, data,)
        if isbits(data) || (isa(data, Array) && isbits(eltype(data)))
            HDF5Logger.log!(log, slug, data)
        else
            for field in fieldnames(data)
                log_structured!(log, slug * "/" * string(field), getfield(data, field))
            end
        end
    end
    HDF5Logger.log!(log, slug * "time", t)
    log_structured!(log, slug * "data", data)
end
