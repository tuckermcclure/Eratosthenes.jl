"""
    EratosthenesRotations

This module includes several rotations and conversions used by the Eratosthenes
simulation engine. For now, only the basic rotation tools necessary for the job
are included. In time, it will become more comprehensive, documented, and
faster.

"""
module EratosthenesRotations

# Questions:
#
# Is a Quat a type, a vector, or a matrix?
# Use arrays of vectors/quats or 2D arrays? Since nothing's vectorized, it seems perfectly easy to do the former.
# Use StaticArrays for speed (very likely)?

export qrot, aa2q, q2aa, qinv, qpos, qcomp, qdiff, qdot, mrp2q

# Make an in-place, batch-oriented function. We could do this in general for all
# of the functions below, but it's not necessary yet. Is this even the style I
# would want to go forward with?
function qrot!(q::Array{T,N}, v::Array{T,N}) where {T,N}
    @assert(size(q, 1) == 4, "First input must be 4-by-n.")
    @assert(size(v, 1) == 3, "Second input must be 3-by-n.")
    @assert(size(q, 2) == size(v, 2), "Quaternion and vector must have same number of elements.")
    for k = 1:size(v,2)
        c1 =  q[4,k]*v[1,k] + q[3,k]*v[2,k] - q[2,k]*v[3,k]
        c2 = -q[3,k]*v[1,k] + q[4,k]*v[2,k] + q[1,k]*v[3,k]
        c3 =  q[2,k]*v[1,k] - q[1,k]*v[2,k] + q[4,k]*v[3,k]
        c4 = -q[1,k]*v[1,k] - q[2,k]*v[2,k] - q[3,k]*v[3,k]
        v[1,k] = -c4*q[1,k] - c3*q[2,k] + c2*q[3,k] + c1*q[4,k]
        v[2,k] =  c3*q[1,k] - c4*q[2,k] - c1*q[3,k] + c2*q[4,k]
        v[3,k] = -c2*q[1,k] + c1*q[2,k] - c4*q[3,k] + c3*q[4,k]
    end
end

function qrot(q::Array{T,N}, v::Array{T,N}) where {T,N}
    v2 = copy(v)
    qrot!(q, v2)
    return v2
end

function aa2q(theta::T, r_hat::Vector{T}) where {T<:Real}
    ht = 0.5*theta
    q = [sin(ht) * r_hat; cos(ht)]
end

# function aa2q(theta::Vector{T}, r_hat::Array{T,2}) where {T<:Real}
#     q = zeros(T, (4, length(theta)))
#     for k = 1:size(theta,1)
#         q[:,k] = aa2q(theta[k], r_hat[:,k])
#     end
#     return q
# end

function q2aa(q::Vector{T}) where {T<:Real}
    half_theta = acos(max(min(q[4], 1.), -1.))
    if half_theta != 0.
        theta = 2. * half_theta
        r = normalize(q[1:3] / sin(half_theta))
    else
        theta = 0.
        r = [1., 0., 0.]
    end
    return (theta, r)
end

function qinv(q::Vector{T}) where {T<:Real}
    @assert(length(q) == 4, "Input must be a 4-element quaternion.")
    return [-q[1:3]; q[4]]
end

function qpos(q::Vector{T}) where {T<:Real}
    @assert(length(q) == 4, "Input must be a 4-element quaternion.")
    return q[4] < zero(T) ? -q : q
end

function qcomp(a::Vector{T}, b::Vector{T}) where {T<:Real}
    @assert(length(a) == 4, "First input must be a 4-element quaternion.")
    @assert(length(b) == 4, "Second input must be a 4-element quaternion.")
    return [ a[4]*b[1] + a[3]*b[2] - a[2]*b[3] + a[1]*b[4];
            -a[3]*b[1] + a[4]*b[2] + a[1]*b[3] + a[2]*b[4];
             a[2]*b[1] - a[1]*b[2] + a[4]*b[3] + a[3]*b[4];
            -a[1]*b[1] - a[2]*b[2] - a[3]*b[3] + a[4]*b[4]]

end

function qdiff(a::Vector{T}, b::Vector{T}) where {T<:Real}
    @assert(length(a) == 4, "First input must be a 4-element quaternion.")
    @assert(length(b) == 4, "Second input must be a 4-element quaternion.")
    return [-a[4]*b[1] - a[3]*b[2] + a[2]*b[3] + a[1]*b[4];
             a[3]*b[1] - a[4]*b[2] - a[1]*b[3] + a[2]*b[4];
            -a[2]*b[1] + a[1]*b[2] - a[4]*b[3] + a[3]*b[4];
             a[1]*b[1] + a[2]*b[2] + a[3]*b[3] + a[4]*b[4]]
end

function qdot(q::Vector{T}, w::Vector{T}, k=zero(T)) where {T<:Real}
    @assert(length(q) == 4, "First input must be a 4-element quaternion.")
    @assert(length(w) == 3, "Second input must be a 3-element rotation rate.")
    qd = T(0.5) * [               w[3] * q[2] - w[2] * q[3] + w[1] * q[4];
                   -w[3] * q[1]               + w[1] * q[3] + w[2] * q[4];
                    w[2] * q[1] - w[1] * q[2]               + w[3] * q[4];
                   -w[1] * q[1] - w[2] * q[2] - w[3] * q[3]               ]
    if k != zero(T)
        m2 = q[1]^2 + q[2]^2 + q[3]^2 + q[4]^2
        s = k * (one(T) - m2)
        qd = qd + s * q
    end
    return qd
end

function mrp2q(p::Vector{T}, scale=one(T)) where {T<:Real}
    f2  = scale^2
    pm2 = p[1]^2 + p[2]^2 + p[3]^2
    c0  = one(T) / (f2 + pm2)
    c1  = 2scale * c0
    return [c1 * p[1];
            c1 * p[2];
            c1 * p[3];
            c0 * (f2 - pm2)]
end

end
