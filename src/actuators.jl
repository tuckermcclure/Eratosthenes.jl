# For right now, actuators only provide force and torque, and they do so with
# 3-by-2 array. In the future, actuators will provide actuations in various
# "domains", such as CentralBodyForce, BodyForce (located at a point, like a
# thruster), MagneticField (also located somewhere specific). Until then, this
# will keep us moving forward.

mutable struct IdealActuatorCommand
    force_B::Vector{Float64}
    torque_B::Vector{Float64}
end

"""
    IdealActuator

This actuator provides exactly effects (e.g., force and torque) that are
requested from it.

"""
function IdealActuator()

    # Initialize the state based on the default (or overwritten) state,
    # constants, and true situation.
    # function init(t, constants, state, draws)
    #     return state
    # end

    # Outputs whatever effects were commanded. This runs at the minor time step.
    function effects(t, constants, state, draws, command, effects, effects_bus)
        return (BodyForce(command.force_B, [0.; 0.; 0.]),
                BodyTorque(command.torque_B))
    end

    # Create the IdealActuator as a DynamicalModel.
    DynamicalModel("ideal_actuator",
                   #init = init,
                   effects = effects,
                   timing = ModelTiming(0.01),
                   inputs = IdealActuatorCommand([0.; 0.; 0.], [0.; 0.; 0.]))

end
