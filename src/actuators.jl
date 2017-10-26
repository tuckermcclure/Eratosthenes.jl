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

"""
   ReactionWheel

This is a completely basic reaction wheel model. You request a torque, and it
spins the wheel with the opposite torque, providing the requested torque as
an Effect.
"""
function ReactionWheel()

    function derivatives(t, constants, speed, draws, torque_cmd, effects, effects_bus)
        -torque_cmd / constants.moment_of_inertia
    end

    function effects(t, constants, speed, draws, torque_cmd, effects, effects_bus)
        (BodyTorque(constants.rotation_axis * torque_cmd),)
    end

    function sense(t, constants, speed, draws, torque_cmd, effects, effects_bus)
        (speed, speed, true)
    end

    DynamicalModel("reaction_wheel", 
                   constants = ReactionWheelConstants([1.; 0.; 0.], 1.),
                   state = 100.,            # Wheel speed (rad/s)
                   inputs = 0.,             # Requested torque (Nm)
                   effects = effects,
                   derivatives = derivatives,
                   update = sense,
                   timing = ModelTiming(0.02))

end

mutable struct ReactionWheelConstants
    rotation_axis::Vector{Float64}
    moment_of_inertia::Float64
end
