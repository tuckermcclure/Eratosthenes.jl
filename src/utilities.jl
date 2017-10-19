# Create a "generated" function to add together two ModelStates. This results in
# a function for each type that is as fast as a custom + function.
@generated function Base.:+(x::T, y::T) where {T <: ModelState}
    expr = ( :(x.$f + y.$f) for f in fieldnames(T) ) # Make a tuple of expressions to add each field
    :( T($(expr...)) ) # Splat them into an expression for a constructor for a T type
end

# Create a generated function to multiple a ModelState by a scalar.
@generated function Base.:*(x::S, y::T) where {S <: Real, T <: ModelState}
    expr = ( :(x * y.$f) for f in fieldnames(T) ) # Make a tuple of expressions to add each field
    :( T($(expr...)) ) # Splat them into an expression for a constructor for a T type
end

# There are built-in types for UT date-times, but they don't work the way that I
# want. This isn't mutable. Just make a new one when something needs to be
# different.
struct UtcDateTime
    Y::Int
    M::Int
    D::Int
    h::Int
    m::Int
    s::Float64
    # UtcDateTime(v::Vector{Number}) UtcDateTime(v[1], v[2], v[3], v[4], v[5], v[6]);
end

# Create a function to convert a vector to a UtcDateType.
Base.convert(::Type{UtcDateTime}, d::Vector{T}) where {T <: Real} = UtcDateTime(d...);
