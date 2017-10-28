#########
# setk! #
#########

function setk!(x, k_requested, v)
    k = setk!_inner(x, k_requested, v)
    if k > 0
        error("Index ", k_requested, " is out of bounds.")
    end
end
@generated function setk!_inner(x::T, k, v) where {T}
    if T <: Array
        if eltype(T) <: Real
            quote
                if k <= length(x)
                    x[k] = v
                    return 0
                end
                return k-length(x)
            end
        else
            # error("I can't handle setting k for an array of anything other than Reals yet.")
            quote
                for c = 1:length(x)
                    k = setk!_inner(x[c], k, v)
                    if k <= 0
                        return 0
                    end
                end
            end
        end
    elseif !isempty(fieldnames(T))
        expr = Vector{Expr}()
        for f in fieldnames(x)
            push!(expr, quote
                if typeof(x.$f) <: Real # For scalars, we must overwrite the field directly.
                    if k == 1
                        x.$f = v
                        return 0
                    end
                    k -= 1
                else # For anything else, it's assumed that we can recur and set something inside the field.
                    k = setk!_inner(x.$f, k, v)
                    if k <= 0
                        return 0
                    end
                end
            end)
        end
        push!(expr, :( return k ))
        Expr(:block, expr...)
    else
        error("I don't know how to set an index of a ", T, ".")
    end
end

########
# getk #
########

# Get index k of x, which might be a struct of struct of vectors or something.
function getk(x, k_requested)
    r, k = getk_inner(x, k_requested)
    if k > 0
        error("Index ", k_requested, " is out of bounds.")
    end
    return r
end
@generated function getk_inner(x::T, k) where {T}
    if T <: Real
        quote
            if k == 1
                return (x, 0)
            end
            return (0., k-1)
        end
    elseif T <: Array
        if eltype(T) <: Real
            quote
                if k <= length(x)
                    return (x[k], 0)
                end
                return (0., k-length(x))
            end
        else
            # error("I can't handle getting k for an array of anything other than Reals yet.")
            quote
                for c = 1:length(x)
                    r, k = getk_inner(x[c], k)
                    if k <= 0
                        return (r, 0)
                    end
                end
            end
        end
    elseif !isempty(fieldnames(T))
        expr = Vector{Expr}()
        push!(expr, :( r = 0. ))
        for f in fieldnames(x)
            push!(expr, quote
                r, k = getk_inner(x.$f, k)
                if k <= 0
                    return (r, 0)
                end
            end)
        end
        push!(expr, :( return (0., k) ))
        Expr(:block, expr...)
    else
        error("I don't know how to get an index of a ", T, ".")
    end
end

###########
# stackem #
###########

# Stacks reals, arrays, and anything with fieldnames into a new vector, returning the vector.
stackem(y::T) where {T} = stackem!(Vector{Float64}(), y)

# Stacks reals, arrays, and anything with fieldnames into an existing vector. Super fast!
# We write this as an if on the type, because this allows us to do more than multiple dispatch.
# E.g., we can detect a struct by asking for its field names.
@generated function stackem!(x, y::T) where {T}
    if T <: Real
        :( push!(x, y) )
    elseif T <: Array
        if eltype(T) <: Real
            :( append!(x, y) )
        else
            quote
                for c = 1:length(y)
                    stackem!(x, y[c])
                end
                x
            end
        end
    elseif !isempty(fieldnames(T)) # This is nice; we can't do this with multiple dispatch. Do this for + and * and remove the ModelStates thing?
        Expr(:block, ( :( stackem!(x, y.$f) ) for f in fieldnames(T) )...)
    elseif T <: Void # There's nothing to stack.
        :(x) # :o
    else
        error("Sorry, I don't know how to stack a ", T, ".")
    end
end

# Stack into a preallocated vector, starting at k+1. Returns k, the number elements written.
@generated function stackem!(x, y::T, k) where {T}
    if T <: Real
        quote
            x[k+1] = y
            k+1
        end
    elseif T <: Array
        if eltype(T) <: Real
            quote
                x[k+1:k+length(y)] = y[:]
                k+length(y)
            end
        else
            quote
                for c = 1:length(y)
                    k = stackem!(x, y[c], k)
                end
            end
        end
    elseif !isempty(fieldnames(T)) # This is nice; we can't do this with multiple dispatch. Do this for + and * and remove the ModelStates thing?
        Expr(:block, ( :( k = stackem!(x, y.$f, k) ) for f in fieldnames(T) )...)
    else
        error("Sorry, I don't know how to stack a ", T, ".")
    end
end