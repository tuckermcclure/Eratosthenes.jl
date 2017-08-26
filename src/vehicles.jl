mutable struct BodyConstants{T} <: ModelConstants where {T <: Real}
    m::T # Mass
    I_B::Matrix{T} # Central mass moment of inertia in body coordinates
    r_mb_B::Vector{T} # Position of center of mass wrt body reference point in body coordinates
end

mutable struct BodyState{T} <: ModelState where {T <: Real}
    r_me_I::Vector{T} # Position of center of mass wrt center of mass of earth in ICRF's coordinates
    v_me_I::Vector{T} # Rate of change of above (as viewed from the inertia frame)
    q_BI::Vector{T}   # Attitude quaternion of body frame wrt ICRF
    ω_BI_B::Vector{T} # Rotation rate of body frame wrt ICRF, in body coordinates
end

mutable struct BodyTruth{T} <: ModelTruth where {T <: Real}
    r_me_I::Vector{T} # Position of center of mass wrt center of mass of earth in ICRF's coordinates
    v_me_I::Vector{T} # Rate of change of above (as viewed from the inertia frame)
    q_BI::Vector{T}   # Attitude quaternion of body frame wrt ICRF
    ω_BI_B::Vector{T} # Rotation rate of body frame wrt ICRF, in body coordinates
    b_B::Vector{T}    # Magnetic field vector
end

"""
Create a DynamicalModel configured to act like a rigid body.
"""
function Body()

    # Configure the body based on any constants or whatever from the setup phase
    # so that the state is ready for simulation to begin. Also, return the
    # initial state and truth so that the sim knows what it will be working
    # with.
    function init(t::Float64, constants::BodyConstants, state::BodyState, draws::Any)
        body_truth = BodyTruth(state.r_me_I, state.v_me_I, state.q_BI, state.ω_BI_B, [1., 0., 0.])
        return (state, body_truth)
    end

    # Calculate the derivatives of the given state.
    function derivatives(t, constants, state, draws, planet, forces)

        # Extract
        q_BI   = state.q_BI
        ω_BI_B = state.ω_BI_B
        f_a_B  = forces[:,1]
        τ_a_B  = forces[:,2]

        # Get the environmental forces.
        g_I = gravity(state.r_me_I, planet.mu)

        # Assemble all of the B-frame forces.
        f_B = f_a_B
        τ_B = τ_a_B

        # Assemble the derivatives.
        a_me_I     = qrot(qinv(q_BI), f_B) / constants.m + g_I
        q_BI_dot   = qdot(q_BI, ω_BI_B)
        ω_BI_B_dot = constants.I_B \ (τ_B - ω_BI_B × (constants.I_B * ω_BI_B))

        # Create the derivative of the state in a way that the simulation can
        # understand.
        return BodyState(state.v_me_I, a_me_I, q_BI_dot, ω_BI_B_dot)

    end

    # Returns the truth about this vehicle in the current situation.
    function step(t, constants, state, draws, planet)

        # Get the Julian date and year fraction.
        # JD = t / (24.*60.*60.) + planet.JD_0
        # Y  = t / (24.*60.*60.*365.25) + planet.igrf_Y_0

        # Get the position in ECEF coordinates.
        # r_me_E = qrot(qIE(JD, epoch=:j2000), state.r_me_I)
        # b_E    = igrf12(planet.igrf, r_ME_E, Y)
        b_E = [0., 0., 0.]

        # Assemble the truth.
        return BodyTruth(state.r_me_I, state.v_me_I, state.q_BI, state.ω_BI_B, b_E)

    end

    Body("body",
         init,
         derivatives,
         step,
         nothing, # shutdown
         0.01,    # maximum-allowable time step
         0.,      # start time
         BodyConstants(50., 5.*eye(3), [0., 0., 0.]),
         BodyState([6378137., 0., 0.], # position
                   [0., 7600., 0.],    # velocity
                   [0., 0., 0., 1.],   # attitude
                   [0., 0., 0.]))      # rotation rate
end

mutable struct Vehicle
    name::String
    body::DynamicalModel
    sensors::Array{DynamicalModel,1}
    software::Array{DynamicalModel,1}
    actuators::Array{DynamicalModel,1}
    Vehicle(name, body, sensors, software, actuators) = new(name, body, sensors, software, actuators);
    Vehicle() = Vehicle("default_vehicle",
                        Body(),
                        Array{DynamicalModel,1}(0),
                        Array{DynamicalModel,1}(0),
                        Array{DynamicalModel,1}(0));
end
