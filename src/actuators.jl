# For right now, actuators only provide force and torque, and they do so with
# 3-by-2 array. In the future, actuators will provide actuations in various
# "domains", such as CentralBodyForce, BodyForce (located at a point, like a
# thruster), MagneticField (also located somewhere specific). Until then, this
# will keep us moving forward.

"""
    IdealActuator

This actuator provides exactly the force and torque that are requested from it.

"""
function IdealActuator()

    # Initialize the state based on the default (or overwritten) state,
    # constants, and true situation.
    function init(t, constants, state, draws, truth)
        return (nothing, # state
                nothing, # measurement
                zeros(3, 2)) # command
    end

    # Output forces at the minor timestep.
    function actuate(t, constants, state, draws, command, truth, whole_truth)
        return command
    end

    # Create the IdealActuator as a DynamicalModel.
    DiscreteActuator("ideal_actuator",
                     init,
                     nothing, # step
                     nothing, # sense
                     actuate,
                     0.01,    # time step
                     0.,      # start time
                     nothing, # constants
                     nothing, # state
                     nothing) # rand

end
