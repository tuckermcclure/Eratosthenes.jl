"""
    IdealPDController

This piece of software relies on a truth sensor and an ideal actuator to align
the spacecraft with the ICRF.

"""
function IdealPDController()

    function step(t, constants, state, inputs, outputs_bus,
                  inputs_bus) # for output only

        # Extract what we care about.
        ω_BI_B = outputs_bus["truth_sensor"].ω_BI_B
        q_BI   = outputs_bus["truth_sensor"].q_BI
        q_TI   = constants # Target quaternion

        # Run the actual algorithm that will fly.
        (f_B, τ_B) = pd_controller(q_TI, q_BI, ω_BI_B)

        # Write out our command for the ideal actuator.
        inputs_bus["ideal_actuator"] = IdealActuatorCommand(f_B, τ_B)

        return (state,      # Updated state (none)
                nothing,    # Outputs
                inputs_bus, # Updated inputs bus (this is not necessary)
                true)       # Active

    end

    DynamicalModel("ideal_pd_controller",
                   init = step,
                   update = step,
                   timing = ModelTiming(0.05),
                   constants = [0.; 0.; 0.; 1.]) # target quaternion

end

# This represents the "real" algorithm under development.
function pd_controller(q_TI, q_BI, ω_BI_B)
    theta, r_B = q2aa(qdiff(q_TI, q_BI))
    τ_B        = 0.2 * theta * r_B - 2 * ω_BI_B
    f_B        = [0.; 0.; 0.]
    return (f_B, τ_B)
end

"""
ReactionWheelController

This piece of software relies on a truth sensor and an ideal actuator to align
the spacecraft with the ICRF.

"""
function ReactionWheelController()

    function step(t, q_TI, state, inputs, outputs_bus,
                inputs_bus) # for output only

        # Extract what we care about.
        q_BI   = outputs_bus["star_tracker"]
        ω_BI_B = outputs_bus["gyro"]

        # Run the actual algorithm that will fly.
        (f_B, τ_B) = pd_controller(q_TI, q_BI, ω_BI_B)

        # Write out our command for the ideal actuator.
        inputs_bus["rw1"] = τ_B[1]
        inputs_bus["rw2"] = τ_B[2]
        inputs_bus["rw3"] = τ_B[3]

        return (state,      # Updated state (none)
                nothing,    # Outputs
                inputs_bus, # Updated inputs bus (this is not necessary)
                true)       # Active

    end

    DynamicalModel("rw_controller",
                init = step,
                update = step,
                timing = ModelTiming(0.05),
                constants = [0.; 0.; 0.; 1.]) # target quaternion

end