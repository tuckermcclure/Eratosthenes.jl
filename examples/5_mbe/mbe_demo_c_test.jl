# mbe_demo_c_test.jl
#
# This script tests the Julia implementation against the C implementation.

# Include our custom models.
include("models.jl")

# We work inside a module because it's more flexible than the main workspace.
module MBEDemoCTest

# Use/import the modules we'll need.
using ..MBEModels  # The MBEModels module included above (and "above" this module)
using Random

# Set some constants (we could randomize these too...).
I  = 5.
κc = 0.5
μc = 1.
ρ  = 2.
α  = 10.

# We'll do this many random tests.
n = 100

# Connect to the library containing our C version of the controller.
c_lib = Libdl.dlopen("windows/lib-reuc.dll")
c_fcn = Libdl.dlsym(c_lib, :reuc)

try

    # Creat a place to store the differences.
    Δτ = zeros(2, n)

    # Seed the random number generator so that this test is repeatable.
    Random.seed!(1)

    for k = 1:n

        # Create a random state and target.
        q_TI   = normalize(randn(4))
        q_BI   = normalize(randn(4))
        ω_BI_B = [randn(1); randn(1); 0.]

        # Run the Julia implementation.
        τ_B = MBEModels.rate_controller(I, κc, μc, ρ, α, q_TI, q_BI, ω_BI_B)

        # Run the C implementation.
        τ_B_c = [0.; 0.; 0.]
        ccall(c_fcn, # Function to call
            Nothing, # No return value
            # Function interface:
            (Float64, Float64, Float64, Float64, Float64, # Parameters
            Ptr{Float64}, Ptr{Float64}, Ptr{Float64}, # Quaternions and rate
            Ptr{Float64}), # Output torque
            # Actual values to pass:
            I, κc, μc, ρ, α,
            q_TI, q_BI, ω_BI_B,
            τ_B_c)

        # Store the differences.
        Δτ[:,k] = τ_B[1:2] - τ_B_c[1:2]

    end

    display(Δτ)
    println("Maximum difference in torque: ", maximum(Δτ[:]), " Nm")

finally
    Libdl.dlclose(c_lib) # Close out the library.
end

end # module
