###########
# Effects #
###########

abstract type Effect end # What's the virtue of having an abstract type for this?

struct NoEffect <: Effect end
struct Gravity <: Effect; specific_force::Vector{Float64}; end
struct BodyForce <: Effect; force::Vector{Float64}; position::Vector{Float64}; end
struct BodyTorque <: Effect; torque::Vector{Float64}; end

mutable struct BodyStateEffect{T <: Real} <: Effect
    r_be_I::Vector{T} # Position of center of mass wrt center of mass of earth in ICRF's coordinates
    v_be_I::Vector{T} # Rate of change of above (as viewed from the inertia frame)
    q_BI::Vector{T}   # Attitude quaternion of body frame wrt ICRF
    ω_BI_B::Vector{T} # Rotation rate of body frame wrt ICRF, in body coordinates
end
BodyStateEffect() = BodyStateEffect(zeros(3), zeros(3), [0.; 0.; 0.; 1.], zeros(3))

#############
# Functions #
#############

# Find a single instance of an effect in a dictionary of effects.
function find_effect(effects_bus::Dict{String, Tuple}, null_effect::T) where T
    for effects in values(effects_bus) # effects contains tuple of Effects from each component
        index = findfirst(e->isa(e, T), effects)
        if !iszero(index)
            return (effects[index], true)
        end
    end
    return (null_effect, false)
end

# Find a single instance of an effect in a vector of effects.
function find_effect(effects::Vector, null_effect::T) where T
    index = findfirst(e->isa(e, T), effects)
    if !iszero(index)
        return (effects[index], index)
    end
    return (null_effect, 0)
end

# function find_effect(all_effects::Dict{String, Tuple}, effect_type::Type)
#     for effects in values(all_effects)
#         index = findfirst(e->isa(e, effect_type), effects)
#         if !iszero(index)
#             return effects[index]
#         end
#     end
#     return NoEffect()
# end
#
# function find_effects(all_effects::Dict{String, Tuple}, effect_type::Type)
#     for effects in values(all_effects)
#         index = findfirst(e->isa(e, effect_type), effects)
#         if !iszero(index)
#             return effects[index]
#         end
#     end
#     return NoEffect()
# end

# # Assumes effects is an iterable array and that the + method exist for T.
# function accumulate(x::T, effects) where T
#     for effect in Iterators.filter(e -> isa(e, T), effects)
#         x += effect
#     end
#     return x
# end
#
# # Allows a user to define a function to accumulate the type into another type
# # (e.g., to extract a vector).
# function accumulate(x, effects, effect_type::Type, f = e->e, args...)
#     for effect in Iterators.filter(e -> isa(e, effect_type), effects)
#         x += f(effect, args...)
#     end
#     return x
# end
#
# # Acts in place and prevents unnecessary creation of vectors.
# function accumulate!(x::Vector, effects, effect_type::Type, f = e->e, args...)
#     for effect in Iterators.filter(e -> isa(e, effect_type), effects)
#         x[:] += f(effect, args...)
#     end
# end

# Acts in place and prevents unnecessary creation of vectors.
function accumulate!(x::Vector, effects_bus::Dict, effect_type::Type, f = e->e, args...)
    for effects in values(effects_bus)
        for effect in effects
            if isa(effect, effect_type)
                x[:] += f(effect, args...)
            end
        end
    end
end

# function accumulate(x, effects_bus::Dict{String, Tuple}, effect_type::Type, f = e->e, args...)
#     return accumulate(x, flatten(values(effects_bus)), effect_type, f, args...)
# end
