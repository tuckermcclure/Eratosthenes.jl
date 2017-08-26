module EratosthenesExampleSoftware

using Eratosthenes

# Bring in the rotations (used by the pd_controller).
include(joinpath("..", "src", "EratosthenesRotations.jl"))
using .EratosthenesRotations

# Bring in the UDP library (used by the PITL mode).
include(joinpath("utilities", "UDPConnections.jl"))
using .UDPConnections

# Let anyone use this controller.
export StarTrackerAndGyroController

# This represents the "real" algorithm under development.
function pd_controller(gains, q_TI, q_BI, ω_BI_B)
    theta, r_B = q2aa(qdiff(q_TI, q_BI))
    τ_B        = gains[1] * theta * r_B - gains[2] * ω_BI_B
    f_B        = [0.; 0.; 0.]
    return (f_B, τ_B)
end

# Julia-in-the-loop, software-in-the-loop, and hardware-in-the-loop
@enum StarTrackerAndGyroControllerMode jitl=0 sitl pitl

# The StarTrackerAndGyroController can be run in 3 modes and needs to store that
# mode and the UDP connection.
mutable struct StarTrackerAndGyroControllerConstants
    target::Vector{Float64}
    mode::StarTrackerAndGyroControllerMode
    conn::UDPConnection
    StarTrackerAndGyroControllerConstants(target = [0.; 0.; 0.; 1.],
                                          mode   = jitl,
                                          conn   = UDPConnection()) =
                                         new(target, mode, conn)
end

"""
    StarTrackerAndGyroController
"""
function StarTrackerAndGyroController()

    # Create a state with a (potential) UDP connection. During setup, the
    # target_ip, send_to port, and listen_on ports can be overwritten.
    constants = StarTrackerAndGyroControllerConstants(
                    [0.; 0.; 0.; 1.], # Target quaternion
                    jitl,             # Default operation mode
                    UDPConnection(ip"192.168.1.3", 2000, 2001))

    # This function initializes any dependencies and provides initial "inputs"
    # and "outputs".
    function init(t, constants, state, args...)
        if constants.mode == pitl
            println("Connecting to remote computer.")
            udp_connect(constants.conn)
        end
        return (state,
                nothing, # inputs that I accept (anyone can write to these)
                nothing) # my outputs (only I can write to these)
    end

    # This function is called each time the controller trigger time is reached.
    # It should update the state using its inputs and the outputs from other
    # software or sensors. It should write to its own outputs and to the inputs
    # of any other software or of actuators.
    function step(t, constants, state, measurements, my_inputs, outputs, inputs, commands)

        # Extract what we care about.
        ω_BI_B = measurements["gyro"]
        q_BI   = measurements["star_tracker"]
        q_TI   = constants.target
        gains  = [0.2; 2.]

        # Run the algorithm under development, the C version, or the C version
        # when running on the flight computer.
        if constants.mode == jitl

            f_B, τ_B = pd_controller(gains, q_TI, q_BI, ω_BI_B)

        # Call the C version of the algorithm directly.
        elseif constants.mode == sitl

            # Create array for the outputs. These will be written to by the C
            # function.
            f_B   = [0.; 0.; 0.]
            τ_B   = [0.; 0.; 0.]
            ccall((:pd_controller, "3_c_code/windows/lib-pd-controller"),
                  Void,
                  (Ptr{Float64}, Ptr{Float64}, Ptr{Float64}, Ptr{Float64}, Ptr{Float64}, Ptr{Float64}),
                  gains, q_TI, q_BI, ω_BI_B, f_B, τ_B);

        # Send the inputs to the flight computer and wait for its outputs.
        elseif constants.mode == pitl

            # Send over the status and other inputs to the flight software.
            udp_send(constants.conn, 1, gains, q_TI, q_BI, ω_BI_B)

            # Give it just a moment to respond. (Sometimes it gets stuck if it
            # goes to receive too soon.)
            sleep(0.001)

            # Wait for the data to arrive. The inputs here serve as examples for
            # the function, enabling it to know how to write to the outputs.
            f_B_tuple = (0., 0., 0.)
            τ_B_tuple = (0., 0., 0.)
            f_B_tuple, τ_B_tuple = udp_receive(constants.conn, f_B_tuple, τ_B_tuple)

            # Expand the tuples into vectors.
            f_B = [f_B_tuple...]
            τ_B = [τ_B_tuple...]

        else
            error("Unknown mode.")
        end

        # Write out our command for the ideal actuator.
        commands["ideal_actuator"] = [f_B τ_B]

        return (state, nothing, inputs, commands)

    end

    # Tell the target that we're done, and then close the socket.
    function shutdown(t, constants, state)
        println("Shutting down the StarTrackerAndGyroController.")
        if constants.mode == pitl
            # Send a status of 0 to indicate that things are over now.
            udp_send(constants.conn, 0)
            udp_close(constants.conn)
        end
    end

    Software("star_tracker_and_gyro_controller",
             init,
             step,
             shutdown,
             0.05,      # Sample rate
             0.,        # Next trigger time
             constants,
             nothing)   # state

end

end
