# We'll use a module to store all of our models.
module MBEModels

# This module will naturally need the types defined in Eratosthenes.
using Eratosthenes

# We'll also use the quaternion stuff that's conveniently included with Eratosthenes.
include(joinpath(Pkg.dir("Eratosthenes"), "src", "EratosthenesRotations.jl"))
using .EratosthenesRotations # Only used for qdiff

# For PITL, we'll need UDP stuff.
include(joinpath(Pkg.dir("Eratosthenes"), "examples", "utilities", "UDPConnections.jl"))
using .UDPConnections

# Export things that need to be available for the user.
export ReducedEffortUnderactuatedController, REUCConstants
export ReducedEffortUnderactuatedControllerSITL, REUCSITLConstants
export ReducedEffortUnderactuatedControllerPITL, REUCPITLConstants

# Create a type to store the constants the controller will need.
mutable struct REUCConstants
    q_TI::Vector{Float64} # Quaternion representing target orientation of body wrt ICRF
    κc::Float64 # Control gain
    μc::Float64 # Control gain
    ρ::Float64  # Control gain
    α::Float64  # Control gain
    I::Float64  # Moment of inertia
end

###############
# Control Law #
###############

# Create the rate control law.
function rate_controller(I, κc, μc, ρ, α, q_TI, q_BI, ω_BI_B)
    
    # Parameterize the rotation rate. Note that ω_BI_B[3] is presumed to be 0.
    ω = ω_BI_B[1] + im * ω_BI_B[2]

    # Turn the target orientation quaternion and current orientation into
    # the z-w parameters. This comes from:
    # 
    # Tsiotras, P., and Longuski, J. M., “A New Parameterization of the
    # Attitude Kinematics,” Journal of the Astronautical Sciences, Vol. 43,
    # No. 3, 1995, pp. 243–262. 
    # 
    # Note that this parameterization fails for the "upside down" case. We can 
    # deal with that later in the development.
    q  = qdiff(q_BI, q_TI)
    z  = 2 * atan2(q[3], q[4]) # 2 * atan(q[3]/q[4])
    w1 = (q[2] * q[3] + q[4] * q[1]) / (q[4]^2 + q[3]^2)
    w2 = (q[4] * q[2] - q[1] * q[3]) / (q[4]^2 + q[3]^2)
    w  = w1 + im * w2
    
    # And calculate its rate of change (Eq. 13a).
    w_dot = 0.5 * ω + 0.5 * conj(ω) * w^2

    # Form the ratio that separates the phase space into good and bad 
    # regions (Eq. 17).
    v = real(w*conj(w))
    η = z / v

    # Form the gains (Eq. 21a and b). These "destabilize" when in the bad 
    # region and drive the system to equilibrium when in the good regions.
    temp = atan(ρ * (1 - η^2))
    κ = 2. * κc / π * temp
    μ = μc / π * temp + 0.5 * μc
    
    # Form the desired rate (Eq. 19).
    ω_d = -κ * w - im * μ * (z/conj(w))

    # Form the rate error (Eq. 40).
    e = ω - ω_d

    # Eq. 42
    e_over_w = e/w
    η_dot = -μ * η + κ * (1 + v) * η + imag(e_over_w) - (1 + v) * η * real(e_over_w)

    # Eq. 37a
    temp  = ρ / (1 + ρ^2 * (1 - η^2)^2) * η * η_dot
    κ_dot = -4 * κc / π * temp
    μ_dot = -2 * μc / π * temp

    # Calculate the rate of change of the desired rate (Eq. 36).
    ω_d_dot = (-κ_dot * w - κ * w_dot - im * μ_dot * η * w 
               - im * μ * η_dot * w - im * μ * η * w_dot)

    # Create the feedback control torque (Eq. 41).
    u = ω_d_dot - α * (ω + κ * w + im * μ * η * w)

    # Turn the torque parameterization into a torque command.
    return τ_B = [I * real(u); I * imag(u); 0.]

end

########################
# Julia Implementation #
########################

# Create a function that constructs a DynamicalModel to act as our controller.
function ReducedEffortUnderactuatedController()

    # Create the model's update function to read from the sensors, determine the
    # desired rate, determine the torque to achieve the desired rate, and send the
    # torque command to the actuators.
    function step(t, constants, state, inputs, outputs_bus, inputs_bus)

        # Extract the attitude and rate.
        ω_BI_B = outputs_bus["truth_sensor"].ω_BI_B
        q_BI   = outputs_bus["truth_sensor"].q_BI

        # Run the controller.
        τ_B = rate_controller(constants.I, 
                              constants.κc, 
                              constants.μc, 
                              constants.ρ, 
                              constants.α, 
                              constants.q_TI, 
                              q_BI, ω_BI_B)

        # Send the torque command to the actuators.
        inputs_bus["ideal_actuator"] = IdealActuatorCommand([0.; 0.; 0.], τ_B) # Torque only (no force)

        # Return a bunch of things.
        return (state,      # Updated state (none)
                nothing,    # Outputs
                inputs_bus, # Updated inputs bus
                true)       # Active

    end
    
    # Create the default set of constants needed by the controller.
    constants = REUCConstants([0.; 0.; 0.; 1.], 0.25, 0.5, 2., 10., 5.)

    # Create the model.
    DynamicalModel("controller",
                   init = step,
                   update = step,
                   constants = constants,
                   timing = ModelTiming(0.05))

end # model constructor

########################
# Software-in-the-Loop #
########################

# We've implemented a new model to run the C implementation. We could instead
# have used the model about and added an option to the constants for which 
# version to run (Julia, SITL, PITL), but that would make the basic model
# more difficult to read. To keep it simple for the sake of this demo, we've
# opted to take the approach of keeping SITL and PITL as separate models, at
# the expense of a little redundant code.
# 
# The C implementation is in reuc.c. See that file for build details.

# Create the constants for SITL, which include the normal parameters as well
# as the C library to call.
mutable struct REUCSITLConstants
    c_lib::Ptr{Void} # Points to library
    c_fcn::Ptr{Void} # Points to function in library
    parameters::REUCConstants # Normal parameters
end

# Create a function that constructs a DynamicalModel that will call our
# C code  version of the controller.
function ReducedEffortUnderactuatedControllerSITL()

    # This function gets called when the simulation is starting up. It's
    # a good place to load the C library.
    function startup(t, constants, state)
        if is_windows()
            constants.c_lib = Libdl.dlopen("windows/lib-reuc.dll")
        else
            constants.c_lib = Libdl.dlopen("linux/lib-reuc.so")
        end
        constants.c_fcn = Libdl.dlsym(constants.c_lib, :reuc)
    end

    # This function gets called when the simulation is over, even if
    # there was an error. We can use it to close out the library.
    function shutdown(t, constants, state)
        Libdl.dlclose(constants.c_lib)
    end

    # Create the model's update function to read from the sensors, determine the
    # desired rate, determine the torque to achieve the desired rate, and send the
    # torque command to the actuators.
    function step(t, constants, state, inputs, outputs_bus, inputs_bus)

        # Extract the attitude and rate.
        ω_BI_B = outputs_bus["truth_sensor"].ω_BI_B
        q_BI   = outputs_bus["truth_sensor"].q_BI

        # Run the controller.
        τ_B = [0.; 0.; 0.]
        ccall(constants.c_fcn, # Function to call
              Void, # No return value
              # Function interface:
              (Float64, Float64, Float64, Float64, Float64, # Parameters
               Ptr{Float64}, Ptr{Float64}, Ptr{Float64}, # Quaternions and rate
               Ptr{Float64}), # Output torque
              # Actual values to pass:
              constants.parameters.I, constants.parameters.κc, constants.parameters.μc,
              constants.parameters.ρ, constants.parameters.α, 
              constants.parameters.q_TI, q_BI, ω_BI_B,
              τ_B)

        # Send the torque command to the actuators.
        inputs_bus["ideal_actuator"] = IdealActuatorCommand([0.; 0.; 0.], τ_B) # Torque only (no force)

        # Return a bunch of things.
        return (state,      # Updated state (none)
                nothing,    # Outputs
                inputs_bus, # Updated inputs bus
                true)       # Active

    end
    
    # Create the default set of constants needed by the controller.
    constants = REUCSITLConstants(
        Ptr{Void}(0), # C lib
        Ptr{Void}(0), # C function
        REUCConstants([0.; 0.; 0.; 1.], 0.25, 0.5, 2., 10., 5.))

    # Create the model.
    DynamicalModel("controller",
                   startup = startup,
                   shutdown = shutdown,
                   init = step,
                   update = step,
                   constants = constants,
                   timing = ModelTiming(0.05))

end # SITL model constructor


#########################
# Processor-in-the-Loop #
#########################

# See reuc-pitl.c.

# Create the constants for SITL, which include the normal parameters as well
# as the C library to call.
mutable struct REUCPITLConstants
    conn::UDPConnection
    parameters::REUCConstants # Normal parameters
end

# Create a function that constructs a DynamicalModel that will call our
# C code  version of the controller.
function ReducedEffortUnderactuatedControllerPITL()

    # This function gets called when the simulation is starting up. It's
    # a good place to load the C library.
    function startup(t, constants, state)
        println("Connecting to target.")
        udp_connect(constants.conn)
    end

    # This function gets called when the simulation is over, even if
    # there was an error. We can use it to close out the library.
    function shutdown(t, constants, state)
        println("Disconnecting from target.")
        udp_send(constants.conn, 0)
        udp_close(constants.conn)
    end

    # Create the model's update function to read from the sensors, determine the
    # desired rate, determine the torque to achieve the desired rate, and send the
    # torque command to the actuators.
    function step(t, constants, state, inputs, outputs_bus, inputs_bus)

        # Extract the attitude and rate.
        ω_BI_B = outputs_bus["truth_sensor"].ω_BI_B
        q_BI   = outputs_bus["truth_sensor"].q_BI

        # Run the controller.
        udp_send(constants.conn, 1, 
            constants.parameters.I, constants.parameters.κc, constants.parameters.μc,
            constants.parameters.ρ, constants.parameters.α, 
            constants.parameters.q_TI, q_BI, ω_BI_B)
        sleep(0.001)
        τ_B_tuple = (0., 0., 0.)
        τ_B_tuple, = udp_receive(constants.conn, τ_B_tuple)
        τ_B = [τ_B_tuple...]

        # Send the torque command to the actuators.
        inputs_bus["ideal_actuator"] = IdealActuatorCommand([0.; 0.; 0.], τ_B) # Torque only (no force)

        # Return a bunch of things.
        return (state,      # Updated state (none)
                nothing,    # Outputs
                inputs_bus, # Updated inputs bus
                true)       # Active

    end
    
    # Create the default set of constants needed by the controller.
    constants = REUCPITLConstants(
        UDPConnection(ip"192.168.1.3", 2000, 2001),
        REUCConstants([0.; 0.; 0.; 1.], 0.25, 0.5, 2., 10., 5.))

    # Create the model.
    DynamicalModel("controller",
                   startup = startup,
                   shutdown = shutdown,
                   init = step,
                   update = step,
                   constants = constants,
                   timing = ModelTiming(0.05))

end # PITL model constructor

end # module
