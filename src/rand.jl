mutable struct RandSource
    f::Function
    n::Int64
    RandSource(f::Function, n::Int64) = new(f, n)
    RandSource(; f::Function=randn, n::Int64=1) = new(f, n)
end

MultiRandSource = Tuple{Vector{RandSource}, Bool}

mutable struct RandSpec
    rng::AbstractRNG
    seed::Int64 # RNG seed (when -1, a seed will be chosen at random using the global RNG)
    init::MultiRandSource # Number of draws necessary for init function
    derivatives::MultiRandSource # Number of draws necessary for derivatives function
    step::MultiRandSource # Number of draws necessary for step function
    sense::MultiRandSource
    actuate::MultiRandSource
    RandSpec(; rng=MersenneTwister(0), seed=-1, init=0, derivatives=0, step=0, sense=0, actuate=0) =
        new(rng, seed, init, derivatives, step, sense, actuate)
end

# convert(::Type{RandSource}, ::Void) = RandSource(n=0)
convert(::Type{RandSource}, n::Int64) = RandSource(n=n)

# convert(::Type{MultiRandSource}, ::Void) = (Vector{RandSource}(), true)
convert(::Type{MultiRandSource}, r::RandSource) = ([r], true)
convert(::Type{MultiRandSource}, n::Int64) = (n > 0 ? [RandSource(n=n)] : Vector{RandSource}(), true)
convert(::Type{MultiRandSource}, t::Tuple{Function, Int64}) = ([RandSource(t...)], true)
convert(::Type{MultiRandSource}, vt::Vector{T}) where {T<:Tuple{Function, Int64}} = ([RandSource(t...) for t in vt], false)

convert(::Type{RandSpec}, ::Void) = RandSpec()

function seed(r::RandSpec, force_seed::Bool = false)
    if force_seed || r.seed < 0
        srand(r.rng, rand(1:1000000))
    else
        srand(r.rng, r.seed)
    end
end

function draw(rand_spec::RandSpec, field::Symbol)
    mrs      = getfield(rand_spec, field)
    minimize = mrs[2]
    draws    = [r.f(rand_spec.rng, r.n) for r in mrs[1]]
    return minimize ? vcat(draws...) : draws
end
