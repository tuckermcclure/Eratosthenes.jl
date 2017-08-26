"""
    IdealPDController

This piece of software relies on a truth sensor and an ideal actuator to align
the spacecraft with the ICRF.

"""
function IdealPDController()

    function init(args...)
        return (nothing, # state
                nothing, # inputs that I accept (anyone can write to these)
                nothing) # my outputs (only I can write to these)
    end

    function step(t, constants, state, measurements, my_inputs, outputs, inputs, commands)

        # Extract what we care about.
        ω_BI_B = measurements["truth_sensor"].ω_BI_B
        q_BI   = measurements["truth_sensor"].q_BI
        q_TI   = constants # Target quaternion

        # Run the actual algorithm that will fly.
        (f_B, τ_B) = pd_controller(q_TI, q_BI, ω_BI_B)

        # Write out our command for the ideal actuator.
        commands["ideal_actuator"] = [f_B τ_B]

        return (state, nothing, inputs, commands)

    end

    Software("ideal_pd_controller",
             init,
             step,
             nothing, # shutdown
             0.05,
             0.,
             [0.; 0.; 0.; 1.], # constants: just a target quaternion
             nothing)          # state

end

# This represents the "real" algorithm under development.
function pd_controller(q_TI, q_BI, ω_BI_B)
    theta, r_B = q2aa(qdiff(q_TI, q_BI))
    τ_B        = 0.2 * theta * r_B - 2 * ω_BI_B
    f_B        = [0.; 0.; 0.]
    return (f_B, τ_B)
end
