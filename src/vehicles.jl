mutable struct BodyConstants{T <: Real}
    m::T # Mass
    I_B::Matrix{T} # Central mass moment of inertia in body coordinates
    r_mb_B::Vector{T} # Position of center of mass wrt body reference point in body coordinates
end

mutable struct BodyState{T} <: ModelState where {T <: Real}
    r_be_I::Vector{T} # Position of center of mass wrt center of mass of earth in ICRF's coordinates
    v_be_I::Vector{T} # Rate of change of above (as viewed from the inertia frame)
    q_BI::Vector{T}   # Attitude quaternion of body frame wrt ICRF
    ω_BI_B::Vector{T} # Rotation rate of body frame wrt ICRF, in body coordinates
end

"""
Create a DynamicalModel configured to act like a rigid body.
"""
function Body()

    # Bodies are the top-lever dynamical objects; they produce effects for
    # others to consume and will see all of the effects of others in their
    # derivatives fuction.
    function effects(t, constants, state, draws)
        return (BodyStateEffect(state.r_be_I, state.v_be_I, state.q_BI, state.ω_BI_B),)
    end

    # Calculate the derivatives of the given state as acted on by the given
    # forces. Note that the effects used must be compatible with the models
    # expected to consume them. This Body models consumes Gravity, BodyForce,
    # and BodyTorque at the moment.
    function derivatives(t, constants, state, draws, effects, effects_bus)

        # Extract
        q_BI   = state.q_BI
        ω_BI_B = state.ω_BI_B

        # Body-fixed magnetic dipole

        # Torque due to magnetic dipole in planet's field
        #b_E = filter(e -> isa(e, MagneticField), effects)

        # Gather up the various forces and torques.

        # Gravity (special case field specific force)
        g_I = zeros(3)
        accumulate!(g_I, effects, Gravity, g -> g.specific_force)

        # Torques
        τ_B = zeros(3)
        accumulate!(τ_B, effects, BodyTorque, τ -> τ.torque)

        # Body-fixed forces
        f_B = zeros(3)
        for e in values(effects)
            for f in filter(f -> isa(f, BodyForce), e)
                f_B[:] += f.force
                τ_B[:] += (f.position - constants.r_mb_B) × f.force
            end
        end

        # Assemble the derivatives.
        a_me_I     = qrot(qinv(q_BI), f_B) / constants.m + g_I
        q_BI_dot   = qdot(q_BI, ω_BI_B)
        ω_BI_B_dot = constants.I_B \ (τ_B - ω_BI_B × (constants.I_B * ω_BI_B))

        # Create the derivative of the state in a way that the simulation can
        # understand.
        return BodyState(state.v_be_I, a_me_I, q_BI_dot, ω_BI_B_dot) # Pretends that b is m. TODO: Add rotation once we actually treat with the center of mass.

    end

    DynamicalModel(
        "body",
        effects = effects,
        derivatives = derivatives,
        timing = ModelTiming(0.01),    # maximum-allowable time step
        constants = BodyConstants(50., 5.*eye(3), [0., 0., 0.]),
        state = BodyState([6378137., 0., 0.], # position
                          [0., 7600., 0.],    # velocity
                          [0., 0., 0., 1.],   # attitude
                          [0., 0., 0.]))      # rotation rate
end

mutable struct Vehicle
    name::String
    body::DynamicalModel
    components::Vector{DynamicalModel}
    computers::Vector{Computer}
    Vehicle(name, body, components, computers) = new(name, body, components, computers)
    Vehicle() = Vehicle("default_vehicle",
                        Body(),
                        Vector{DynamicalModel}(0),
                        Vector{Computer}(0))
end
