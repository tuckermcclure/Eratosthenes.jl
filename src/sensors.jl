"""
    Truth Sensor

No matter what the vehicle truth is, this sensor can sense it. It's a useful
debugging tool.

"""
function TruthSensor()
    function init(t, constants, state, draws, truth)
        return (nothing, truth)
    end
    function sense(t, constants, state, draws, truth, whole_truth)
        return truth
    end
    DiscreteSensor("truth_sensor",
                   init,
                   nothing, # step
                   sense,
                   nothing, # shutdown
                   0.02,    # time step
                   0.,      # start time
                   nothing, # constants
                   nothing, # state
                   nothing) # rand
end

"""
    Star Tracker

This generic star tracker always reports the body's orientation (a quaternion)
with a little noise.

"""
function StarTracker()
    function init(t, constants, state, draws, vehicle_truth)
        return (nothing, [0., 0., 0., 1.])
    end
    function sense(t, constants, state, draws, vehicle_truth, truth)
        errors = constants .* draws
        return qcomp(mrp2q(errors), vehicle_truth.q_BI)
    end
    DiscreteSensor("default_star_tracker",
                   init,
                   nothing, # state
                   sense,
                   nothing, # shutdown
                   0.1,     # 10Hz update rate
                   0.,      # begins measuring at t=0
                   [0.001, 0.001, 0.001], # constants: error magnitudes (rad)
                   nothing, # state
                   RandSpec(sense=3)) # rand
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
end
mutable struct GyroState
    bias::Vector{Float64}
    q_SI::Vector{Float64}
    ω::Vector{Float64}
    t::Float64
end

function Gyro()
    function init(t, constants, state, draws, vehicle_truth)
        initial_bias = constants.initial_bias_magnitude * (2. * draws[1:3] - 1.)
        return (GyroState(initial_bias, vehicle_truth.q_BI, vehicle_truth.ω_BI_B, t),
                [0.; 0.; 0.]) # example measurement
    end
    function step(t, constants, state, draws, vehicle_truth, truth)
        Δt     = t - state.t # Would be nice if models had access to this.
        σ_rate = Δt > 0. ? constants.angular_random_walk / √(Δt) : 0. # These could be saved to constants for a fixed time step, which would save us from dividing by zero at the first sample.
        σ_bias = constants.bias_random_walk * √(Δt)
        noise  = σ_rate * draws[1:3]
        Δb     = σ_bias * draws[4:6]
        θ, r   = q2aa(qdiff(vehicle_truth.q_BI, state.q_SI))
        Δθ     = θ * r + (state.bias + 0.5*Δb) * Δt + noise
        ω      = Δθ / Δt
        return GyroState(state.bias + Δb, vehicle_truth.q_BI, ω, t)
    end
    function sense(t, constants, state, draws, vehicle_truth, truth)
        return state.ω
    end
    DiscreteSensor("default_gyro",
                   init,
                   step,
                   sense,
                   nothing, # shutdown
                   0.01,
                   0.,
                   GyroConstants(0.00001, 0.0001, 0.001),
                   GyroState([0.; 0.; 0.], [0.; 0.; 0.; 1.], [0.; 0.; 0.], 0.),
                   RandSpec(init=(rand,3), step=6))
end
