mutable struct BodyConstants{T <: Real}
    m::T # Mass
    I_B::Matrix{T} # Central mass moment of inertia in body coordinates
    r_mb_B::Vector{T} # Position of center of mass wrt body reference point in body coordinates
    Δr::Vector{T} # Random position perturbations (m) (for initialization)
    Δv::Vector{T} # Random velocity perturbations (m/s)
    Δq::Vector{T} # Random orientation perturbations (~rad)
    Δω::Vector{T} # Random rotation rate perturbations (rad/s)
end

mutable struct BodyState{T <: Real} <: ModelState
    r_be_I::Vector{T} # Position of center of mass wrt center of mass of earth in ICRF's coordinates
    v_be_I::Vector{T} # Rate of change of above (as viewed from the inertia frame)
    q_BI::Vector{T}   # Attitude quaternion of body frame wrt ICRF
    ω_BI_B::Vector{T} # Rotation rate of body frame wrt ICRF, in body coordinates
end

"""
Create a DynamicalModel configured to act like a rigid body.
"""
function Body()

    # Upon initialization, Bodies can perturb their nominal states
    # by the specified amount (defaults to 0).
    function init(t, constants, state, draws)
        state.r_be_I += constants.Δr .* draws[1:3]
        state.v_be_I += constants.Δv .* draws[4:6]
        state.q_BI   =  qcomp(mrp2q(constants.Δq .* draws[7:9], 4.), state.q_BI)
        state.ω_BI_B += constants.Δω .* draws[10:12]
        return state
    end

    # Bodies are the top-lever dynamical objects; they produce effects for
    # others to consume and will see all of the effects of others in their
    # derivatives fuction.
    function effects(t, constants, state, draws, implicit)
        return (BodyStateEffect(state.r_be_I, state.v_be_I, state.q_BI, state.ω_BI_B),)
    end

    # Calculate the derivatives of the given state as acted on by the given
    # forces. Note that the effects used must be compatible with the models
    # expected to consume them. This Body models consumes Gravity, BodyForce,
    # and BodyTorque at the moment.
    function derivatives(t, constants, state, draws, implicit, effects, effects_bus)

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
            for f in e
                if isa(f, BodyForce)
                    f_B[:] += f.force
                    τ_B[:] += (f.position - constants.r_mb_B) × f.force
                end
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

    # Create the set of constants used by the rigid body.
    constants = BodyConstants(50., Matrix(Diagonal([5.0; 5; 5])), zeros(3),
                              zeros(3), zeros(3), zeros(3), zeros(3))

    DynamicalModel(
        "body",
        init = init,
        effects = effects,
        derivatives = derivatives,
        timing = ModelTiming(0.01),    # maximum-allowable time step
        constants = constants,
        state = BodyState([6378137., 0., 0.], # position
                          [0., 7600., 0.],    # velocity
                          [0., 0., 0., 1.],   # attitude
                          [0., 0., 0.]),      # rotation rate
        rand = RandSpec(init=12)) # We need 3 Gaussian draws for the init fcn.
end

mutable struct Vehicle
    name::String
    body::DynamicalModel
    components::Vector{DynamicalModel}
    computers::Vector{Computer}
    Vehicle(name, body, components, computers) = new(name, body, components, computers)
    Vehicle() = Vehicle("default_vehicle",
                        Body(),
                        Vector{DynamicalModel}(undef, 0),
                        Vector{Computer}(undef, 0))
end
