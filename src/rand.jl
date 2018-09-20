mutable struct RandSource{F}
    f::F
    n::Int64
    RandSource(f::F, n::Int64) where {F <: Function} = new{F}(f, n)
    RandSource(; f::F = randn, n::Int64 = 1) where {F <: Function} = new{F}(f, n)
end

MultiRandSource = Tuple{Tuple, Bool}

mutable struct RandSpec{R}
    rng::R #AbstractRNG
    seed::Int64 # RNG seed (when -1, a seed will be chosen at random using the global RNG)
    init::MultiRandSource # Number of draws necessary for init function
    effects::MultiRandSource # Number of draws necessary for effects function
    derivatives::MultiRandSource # Number of draws necessary for derivatives function
    update::MultiRandSource # Number of draws necessary for update function
    RandSpec(; rng::R = MersenneTwister(0), seed=-1, init=0, effects=0, derivatives=0, update=0) where {R <: AbstractRNG} =
        new{R}(rng, seed, init, effects, derivatives, update)
end

# convert(::Type{RandSource}, ::Nothing) = RandSource(n=0)
convert(::Type{RandSource}, n::Int64) = RandSource(n=n)

# convert(::Type{MultiRandSource}, ::Nothing) = (Vector{RandSource}(), true)
convert(::Type{MultiRandSource}, r::RandSource) = ((r,), true)
convert(::Type{MultiRandSource}, n::Int64) = (n > 0 ? (RandSource(n=n),) : (), true)
convert(::Type{MultiRandSource}, t::Tuple{Function, Int64}) = ((RandSource(t...),), true)
convert(::Type{MultiRandSource}, vt::Vector{T}) where {T<:Tuple{Function, Int64}} = (collect(RandSource(t...) for t in vt), false)

convert(::Type{RandSpec}, ::Nothing) = RandSpec()

function seed(r::RandSpec, force_seed::Bool = false)
    if force_seed || r.seed < 0
        Random.seed!(r.rng, rand(1:1000000))
    else
        Random.seed!(r.rng, r.seed)
    end
end

function draw(rand_spec::RandSpec, field::Symbol)
    mrs      = getfield(rand_spec, field)
    minimize = mrs[2]
    draws    = [r.f(rand_spec.rng, r.n) for r in mrs[1]]
    return minimize ? vcat(draws...) : draws
end
