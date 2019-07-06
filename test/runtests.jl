using Eratosthenes
using Test
using LinearAlgebra

include(joinpath(dirname(pathof(Eratosthenes)), "EratosthenesRotations.jl"))
using .EratosthenesRotations

########################
# Individual Rotations #
########################

# Test that qcomp and qdiff are consistent.
a = normalize([1., 2., 3., 4.])
b = normalize([2., 2., 1., -2.])
Δθ, r = q2aa(qpos(qdiff(qdiff(a, b), qcomp(a, qinv(b)))))
@test Δθ < 1e-12

##############
# Simulation #
##############

# While this isn't much of a unit test, the following exercises a huge amount of
# the system with nonetheless easily-predictable results. Let's see if things
# work.

# Create an actuator that provides a constant torque.
function Constorquer()
    function effects(t, constants, state, draws, implicit, inputs, effects, effects_bus)
        return (BodyTorque(constants),)
    end
    DynamicalModel("constorquer",
                   effects = effects,
                   constants = [2.; 0.; 0.], # Torque (must be about principal axis)
                   timing = ModelTiming(0.01))
end

# Create a default simulation with only a constorquer. We'll be able to predict
# the end result.
scenario = Scenario()
scenario.sim.t_end = 5.
scenario.vehicles[1].components = [Constorquer()]
simulate(scenario)

# Test that we rolled just as much as expected.
τ             = scenario.vehicles[1].components[1].constants
Δt            = scenario.sim.t_end
I_B           = scenario.vehicles[1].body.constants.I_B
q_expected    = aa2q(0.5 * Δt^2 * norm(I_B \ τ), normalize(τ))
ω_expected    = Δt * (I_B \ τ)

# Calculate the errors.
q     = normalize(scenario.vehicles[1].body.state.q_BI)
ω     = scenario.vehicles[1].body.state.ω_BI_B
Δθ, r = q2aa(qpos(qdiff(q, q_expected)))
Δω    = ω - ω_expected

# How'd we do?
@test Δθ < 1e-12
@test norm(Δω) < 1e-12

###########
# Example #
###########

# If the out directory doesn't exist, make it.
if !isdir("out")
    mkdir("out")
end

# min-scenario
success = false;
try
    include("run-min-scenario-ci.jl")
    global success = true;
catch err
    println("There was an error running the basic example.")
    rethrow(err)
end

@test success == true

# coupled reaction wheel scenario
success = false;
try
    local scenario = setup(joinpath(dirname(pathof(Eratosthenes)), "..", "examples", "scenarios", "reaction-wheel.yaml"))
    scenario.sim.t_end = 1.
    simulate(scenario)
    global success = true;
catch err
    println("There was an error running the reaction wheel example.")
    rethrow(err)
end

@test success == true
