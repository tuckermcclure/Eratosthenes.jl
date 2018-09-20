"""
    Truth Sensor

This sensor can detect all physical effects generated on the vehicle.

"""
function TruthSensor()
    # function init(t, constants, state, draws, inputs, effects, effects_bus)
    #     return state
    # end
    function sense(t, constants, state, draws, inputs, effects, effects_bus)
        body, = find_effect(effects, BodyStateEffect())
        return (state, body, true)
    end
    DynamicalModel("truth_sensor",
                   init = sense,
                   update = sense,
                   outputs = BodyStateEffect(),
                   timing = ModelTiming(0.02))
end

"""
    Star Tracker

This generic star tracker always reports the body's orientation (a quaternion)
with a little noise.

"""
function StarTracker()
    function sense(t, constants, state, draws, inputs, effects, effects_bus)
        errors = constants .* draws
        body, = find_effect(effects, BodyStateEffect())
        return (state, qcomp(mrp2q(errors, 4.), body.q_BI), true)
    end
    DynamicalModel("default_star_tracker",
                   init = sense,
                   update = sense,
                   timing = ModelTiming(0.1), # 10Hz update rate
                   constants = [0.001, 0.001, 0.001], # constants: error magnitudes (rad)
                   outputs = [0.; 0.; 0.; 1.],
                   rand = RandSpec(init=3,update=3)) # rand
end

"""
    Gyro

This is a generic rate-integrating gyro with angular random walk due to noise
and a bias random walk. It doesn't use the state's rotation rate, but actually
differences the quaternion.

"""
mutable struct GyroConstants
    angular_random_walk::Float64
    bias_random_walk::Float64
    initial_bias_magnitude::Float64
    # q_SB::Vector{Float64} # Orientation wrt the body
end
mutable struct GyroState
    bias::Vector{Float64}
    q_SI::Vector{Float64}
    t::Float64
end

function Gyro()
    function init(t, constants, state, draws, inputs, effects, effects_bus)
        initial_bias = constants.initial_bias_magnitude * (2. .* draws[1:3] .- 1.)
        body, = find_effect(effects, BodyStateEffect())
        return (GyroState(initial_bias, body.q_BI, t),
                body.ω_BI_B, # Outputs
                true) # Active?
    end
    function update(t, constants, state, draws, inputs, effects, effects_bus)
        body, found  = find_effect(effects, BodyStateEffect())
        if !found; error("The Gyro model requires a BodyState effect but could not find one."); end
        Δt     = t - state.t # It would be nice if models had access to their own dt.
        σ_rate = Δt > 0. ? constants.angular_random_walk / √(Δt) : 0. # These could be saved to constants for a fixed time step, which would save us from dividing by zero at the first sample.
        σ_bias = constants.bias_random_walk * √(Δt)
        noise  = σ_rate * draws[1:3]
        Δb     = σ_bias * draws[4:6]
        θ, r   = q2aa(qdiff(body.q_BI, state.q_SI))
        Δθ     = θ * r + (state.bias + 0.5*Δb) * Δt + noise
        ω      = Δθ / Δt
        return (GyroState(state.bias + Δb, body.q_BI, t), # Updated state
                ω,                                        # Measurement
                true)                                     # Active?
    end
    DynamicalModel("gyro",
                   init = init,
                   update = update,
                   timing = ModelTiming(0.01),
                   constants = GyroConstants(0.00001, 0.0001, 0.001),
                   state = GyroState([0.; 0.; 0.], [0.; 0.; 0.; 1.], 0.),
                   outputs = [0.; 0.; 0.],
                   rand = RandSpec(init=(rand,3), update=6))
end
