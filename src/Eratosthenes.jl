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

import Base: convert, +, *, accumulate! # Required in order to extend these things.
using Base.Iterators
import YAML
using HDF5Logger

# For now, our rotations are in another module included with this package. We
# might move them into their own package if people find them generally useful.
# To include this module from outside of Eratosthenes, use:
#
#    include(Pkg.dir("Eratosthenes") * "/src/EratosthenesRotations.jl")
#    using .EratosthenesRotations
#
include("EratosthenesRotations.jl")
using .EratosthenesRotations

export UtcDateTime
export RandSpec, RandSource, MultiRandSource, draw # Rand stuff
export ModelTiming, DynamicalModel # Top-level modeling types
export Computer, Vehicle # Containers
export LowEarthOrbit, TruthSensor, StarTracker, Gyro, IdealActuator # Models
export IdealActuatorCommand
export NoEffect, Gravity, BodyTorque, BodyForce
export find_effect, accumulate!
export setup, simulate, mc
export Scenario, SimParams

# Used to tell integrator how to add.
abstract type ModelState end

include("utilities.jl")
include("rand.jl")
include("physics.jl")
include("setup.jl")
include("effects.jl")

mutable struct ModelTiming
    dt::Float64
    t_start::Float64
    t_next::Float64
    count::Int64 # Total number of times this model has triggered
    ModelTiming(dt = 0., t_start = 0., t_next = 0., count = 0) = new(dt, t_start, t_next, count)
end

mutable struct DynamicalModel{F0, FI, FE, FD, FU, FS, DC, DS, DI, DO}

    name::String # Actually used as a key in a dictionary, as well as for logging

    # Functions
    #
    # NOTE: These can't be changed without changing the type of the object since
    # we've parameterized the type for speed. If we later make immutable things
    # out of the mutable things, then we'll remove this parameterization here
    # but keep it for the immutable version for speed there.
    #
    startup::F0 # Called to allow one to set up resources or whatever.
    init::FI # "Tell me what you produce."
    effects::FE # "Produce physical effects."
    derivatives::FD # "Tell me how the state is changing."
    update::FU # "Update yourself for this time step."
    shutdown::FS # Called when the sim ends, even when there's an error.

    timing::ModelTiming

    # Data
    constants::DC # This will be set up during setup and will never change afterwards.
    state::DS # Bonus: return a ModelState to enable "struct addition and multiplication"
    inputs::DI # Inputs accepted by this model (e.g., commands); software can write to these.
    outputs::DO # Outputs produced by this model (e.g., measurements); software can consume these.

    rand::RandSpec # Provide the properties of the random number generator stream you'll need.

end

# Create a convenience constructor with lots of defaults.
DynamicalModel(name;
               startup = nothing,
               init = nothing,
               effects = nothing,
               derivatives = nothing,
               update = nothing,
               shutdown = nothing,
               timing = ModelTiming(),
               constants = nothing,
               state = nothing,
               inputs = nothing,
               outputs = nothing,
               rand = RandSpec()) =
    DynamicalModel(name, startup, init, effects, derivatives, update, shutdown, timing, constants, state, inputs, outputs, rand)

# Convenience constructors
# Body(name, init, effects, derivatives, shutdown, timing, constants, state, rand = RandSpec()) =
#     DynamicalModel(name, init, effects, derivatives, nothing, shutdown, timing, constants, state, nothing, nothing, rand)
# DiscreteSensor(name, init, update, shutdown, timing, constants = nothing, state = nothing, inputs = nothing, outputs = nothing, rand = RandSpec()) =
#     DynamicalModel(name, init, nothing, nothing, update, shutdown, timing, constants, state, inputs, outputs, rand)
# DiscreteActuator(name, init, effects, update, shutdown, timing, constants = nothing, state = nothing, inputs = nothing, outputs = nothing, rand = RandSpec()) =
#     DynamicalModel(name, init, effects, nothing, update, shutdown, timing, constants, state, inputs, outputs, rand)
# Software(name, init, step, shutdown, inputs, outputs, dt, t_start, constants, state) =
#     DynamicalModel(name, init, effects, derivatives, update, shutdown, timing, constants, state, inputs, outputs, rand)
# ConstantModel(name, init, effects, shutdown, constants, rand = RandSpec()) =
#     DynamicalModel(name, init, effects, derivatives, update, shutdown, timing, constants, state, inputs, outputs, rand)
NullModel() = DynamicalModel("nothing")

# Include the other code in no particular order.
include("actuators.jl")
include("planets.jl")
include("sensors.jl")
include("software.jl")
include("computers.jl")
include("vehicles.jl")

# The things below set some defaults that depend on the above.

# Options for global simulation stuff.
mutable struct SimParams
    dt::Float64 # Time step (s)
    t_end::Float64 # End time of simulation (s)
    seed::Int64 # Seed for default random number generator
    date_0::UtcDateTime # UTC date at the beginning of the sim
    log_file::String # Name of HDF5 file to log to
    SimParams() = new(0.01, 10.0, 1, UtcDateTime(2017, 7, 28, 9, 27, 33.0), "")
    SimParams(a,b,c,d,e) = new(a,b,c,d,e)
end

# Top-level options describing what should happen in the sim.
mutable struct Scenario
    sim::SimParams
    environment::DynamicalModel
    vehicles::Array{Vehicle,1}
    Scenario() = new(SimParams(), LowEarthOrbit(), [Vehicle()])
    Scenario(s,e,va) = new(s,e,va)
end

include("simulate.jl")

end
