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
    function effects(t, constants, state, draws, implicit, command, effects, effects_bus)
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

TODO: This should really be a 0-momentum reaction wheel array, because it the
wheels speeds don't affect the body yet. We could make a MomentumEffect that the
body dynamics could consume that would allow it to generically account for
reaction wheels cleanly.

This is a completely basic reaction wheel model. You request a torque, and it
spins the wheel with the opposite torque, providing the requested torque as
an Effect.
"""
function ReactionWheel()

    function derivatives(t, constants, speed, draws, implicit, τ_br_cmd, effects, effects_bus)
        -τ_br_cmd / constants.I_r_G[1]
    end

    function effects(t, constants, speed, draws, implicit, τ_br_cmd, effects, effects_bus)
        body, found  = find_effect(effects, BodyStateEffect())
        if !found
            error("Couldn't find BodyStateEffect.")
        end
        ω_BI_G = qrot(constants.q_GB, body.ω_BI_B)
        τ_br_G = [τ_br_cmd; 0.; 0.] - ω_BI_G × ([constants.I_r_G[1] * speed; 0.; 0.] + constants.I_r_G .* ω_BI_G)
        τ_br_B = qrot(qinv(constants.q_GB), τ_br_G)
        (BodyTorque(τ_br_B),)
    end

    function sense(t, constants, speed, draws, τ_br_cmd, effects, effects_bus)
        (speed, speed, true)
    end

    DynamicalModel("reaction_wheel",
                   constants = ReactionWheelConstants([1.; 1/sqrt(2.); 1/sqrt(2.)], [0.; 0.; 0.; 1.]),
                   state = 100. * 2pi/60., # Wheel speed (rad/s)
                   inputs = 0.,             # Requested torque (Nm)
                   outputs = 100. * 2pi/60., # Measured wheel speed (rad/s)
                   effects = effects,
                   derivatives = derivatives,
                   update = sense,
                   timing = ModelTiming(0.02))

end

mutable struct ReactionWheelConstants
    I_r_G::Vector{Float64}
    q_GB::Vector{Float64}
end

"""
   CoupledReactionWheel

This reaction wheel is modular. The body dynamics don't need to know about it at
all. It uses the differential algebraic equation framework of the simulation
environment so that we don't need to implement the coupled rotor-and-body
dynamics explicitly.

"""
function CoupledReactionWheel()

    # Get the torque on the rotor, coming from motor torque and constraints.
    function torque(τ_br_cmd, constants, state, implicit)
        τ_rb_G = [-τ_br_cmd; implicit[1]; implicit[2]]
        return τ_rb_G
    end

    # Get the derivative of the state, treating the rotor as a free 3DOF body.
    function derivatives(t, constants, state, draws, implicit, τ_br_cmd, effects, effects_bus)
        τ_rb_G     = torque(τ_br_cmd, constants, state, implicit)
        ω_RI_G_dot = constants.I_r_G .\ (τ_rb_G - state.ω_RI_G × (constants.I_r_G .* state.ω_RI_G))
        return CoupledReactionWheelState(ω_RI_G_dot)
    end

    # Ensure that the implicit torque keeps the rotation rates of the rotor wrt
    # the body entirely around the rotor axis (no off-axis rotation of one wrt
    # the other).
    function constraints(t, constants, state, draws, implicit, τ_br_cmd, effects, effects_bus)
        body,  = find_effect(effects, BodyStateEffect())
        ω_BI_G = qrot(constants.q_GB, body.ω_BI_B)
        return state.ω_RI_G[2:3] - ω_BI_G[2:3]
    end

    # The torque on the rotor has an equal and opposite torque on the body.
    function effects(t, constants, state, draws, implicit, τ_br_cmd, effects, effects_bus)

        τ_rb_G = torque(τ_br_cmd, constants, state, implicit)
        τ_br_B = qrot(qinv(constants.q_GB), -τ_rb_G)

        # Check:
        # body,  = find_effect(effects, BodyStateEffect())
        # if body.r_be_I[1] == 1.
        #     ω_BI_G = qrot(constants.q_GB, body.ω_BI_B)
        #     τ_br_G = [τ_br_cmd; 0.; 0.] - ω_BI_G × ([constants.I_r_G[1] * state.ω_RI_G[1]; 0.; 0.] + constants.I_r_G .* ω_BI_G)
        #     τ_br_B_2 = qrot(qinv(constants.q_GB), τ_br_G)
        #     println("implicit torque: ", τ_br_B, "; analytical torque: ", τ_br_B_2)
        # end
        
        (BodyTorque(τ_br_B),)
    end

    # Produce the measurement (the rotor speed) and also reset the rotation to
    # be strictly on-axis (removes buildup of constraint torque noise)
    function update(t, constants, state, draws, τ_br_cmd, effects, effects_bus)
        body,  = find_effect(effects, BodyStateEffect())
        ω_BI_G = qrot(constants.q_GB, body.ω_BI_B)
        state.ω_RI_G[2:3] = ω_BI_G[2:3]
        (state, state.ω_RI_G[1] - ω_BI_G[1], true)
    end

    DynamicalModel("coupled_reaction_wheel",
                   constants = CoupledReactionWheelConstants([1.; 1/sqrt(2.); 1/sqrt(2.)], [0.; 0.; 0.; 1.]),
                   state = CoupledReactionWheelState([100. * 2pi/60.; 0.; 0.]),
                   inputs = 0., # Requested torque on rotor from body about 1 axis of G frame (Nm)
                   outputs = 100. * 2pi/60.,
                   implicit = [0.; 0.], # Implicit variables (constraint torque) (Nm)
                   effects = effects,
                   derivatives = derivatives,
                   constraints = constraints,
                   update = update,
                   timing = ModelTiming(0.02))

end

mutable struct CoupledReactionWheelConstants
    I_r_G::Vector{Float64}
    q_GB::Vector{Float64}
end

mutable struct CoupledReactionWheelState <: ModelState
    ω_RI_G::Vector{Float64}
end
