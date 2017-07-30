"""
    Eratosthenes

This spacecraft simulation environment allows a user to quickly create simulations with different spacecraft, spacecraft components, and simulation options.

```julia
Pkg.clone("https://github.com/tuckermcclure/Eratosthenes.jl.git")
using Eratosthenes
scenario = setup(Scenario(), "my-scenario.json")
simulate(scenario)
```
"""
module Eratosthenes

import Base.convert # Required in order to extend 'convert'.
import JSON
import YAML
using HDF5Logger

# For now, our rotations are in another module included with this package. We
# might move them into their own package if people find them generally useful.
# To include this module from outside of Eratosthenes, use:
#
#    include(Pkg.dir("Eratosthenes") * "/src/EratosthenesRotations.jl")
#    using .EratosthenesRotations
#
include(@__DIR__() * "/EratosthenesRotations.jl")
using .EratosthenesRotations

export UtcDateTime
export RandSpec, RandSource, MultiRandSource, draw

export DynamicalModel
export Body, Software, DiscreteSensor, DiscreteActuator
export Earth
export TruthSensor, StarTracker, Gyro
export IdealActuator
export StarTrackerAndGyroController

export setup, simulate
export Scenario, SimParams

abstract type ModelConstants end # There's no real point to this.
abstract type ModelState end
abstract type ModelRand end # What's the interface going to be here?
abstract type ModelTruth end # What's this do? Truths could be immutable!
abstract type ModelMeasurement end # Measurements should be immuatble too.
abstract type ModelCommand end

# Create a function to add together two ModelStates.
function Base.:+(a::T, b::T) where {T <: ModelState}
    s = deepcopy(a)
    for field in fieldnames(a)
        setfield!(s, field, getfield(a, field) + getfield(b, field))
    end
    return s
end

# Create a function to multiply a ModelState by a scalar.
function Base.:*(a::S, b::T) where {S <: Real, T <: ModelState}
    s = deepcopy(b)
    for field in fieldnames(b)
        setfield!(s, field, a * getfield(b, field))
    end
    return s
end

include("utilities.jl")
include("rand.jl")
include("physics.jl")
include("setup.jl")

mutable struct DynamicalModel
    name::String # Actually used as a key in a dictionary, as well as for logging
    init::Union{Function,Void} # "Tell me what you produce."
    derivatives::Union{Function,Void} # "Tell me how the state is changing."
    step::Union{Function,Void} # "Update yourself for this time step."
    sense::Union{Function,Void} # "Product your measurement."
    actuate::Union{Function,Void} # "Produce forces."
    dt::Float64
    t_next::Float64
    constants::Any # This will be set up during setup and will never change afterwards.
    state::Any # Bonus: return a ModelState to enable "struct addition and multiplication"
    rand::RandSpec # Provide the properties of the random number generator stream you'll need.
end

# Convenience constructors
Body(name, init, derivatives, step, dt, t_next, constants, state, rand = RandSpec()) =
    DynamicalModel(name, init, derivatives, step, nothing, nothing, dt, t_next, constants, state, rand)
DiscreteSensor(name, init, step, measure, dt, t_next, constants = nothing, state = nothing, rand = RandSpec()) =
    DynamicalModel(name, init, nothing, step, measure, nothing, dt, t_next, constants, state, rand)
DiscreteActuator(name, init, step, measure, actuate, dt, t_next, constants = nothing, state = nothing, rand = RandSpec()) =
    DynamicalModel(name, init, nothing, step, measure, actuate, dt, t_next, constants, state, rand)
Software(name, init, step, dt, t_next, constants, state) =
    DynamicalModel(name, init, nothing, step, nothing, nothing, dt, t_next, constants, state, nothing)
ConstantModel(name, init, constants, rand = RandSpec()) =
    DynamicalModel(name, init, nothing, nothing, nothing, nothing, 0., 0., constants, 0., rand) # Use for planet?

# Include the other code, from smallest thing to biggest thing.
include("sensors.jl")
include("actuators.jl")
include("software.jl")
include("vehicles.jl")

# The things below set some defaults that depend on the above.

# Options for global simulation stuff.
mutable struct SimParams
    dt::Float64 # Time step (s)
    t_end::Float64 # End time of simulation (s)
    seed::Int64 # Seed for default random number generator
    date_0::UtcDateTime # UTC date at the beginning of the sim
    log_file::String # Name of HDF5 file to log to
    progress_fcn::Union{Function,Void} # Called at the beginning of each iteration of the sim
    SimParams() = new(0.01, 10.0, 1, UtcDateTime(2017, 7, 28, 9, 27, 33.0), "", nothing)
    SimParams(a,b,c,d,e) = new(a,b,c,d,e)
end

mutable struct Earth # TODO: Make this a ConstantModel?
    mu::Float64
    JD_0::Float64
    igrf_Y_0::Float64
    igrf::Any
    Earth(; mu = 3.986004418e14, JD_0 = 0., igrf_Y_0 = 0., igrf = nothing) = new(mu, JD_0, igrf_Y_0, igrf);
end

# Top-level options describing what should happen in the sim.
mutable struct Scenario
    sim::SimParams
    planet::Any
    vehicles::Array{Vehicle,1}
    Scenario() = new(SimParams(), Earth(), [Vehicle()]);
    Scenario(s,p,va) = new(s,p,va);
end

include("simulate.jl")

end
