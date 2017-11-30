# mbe_demo_mc.jl
# 
# This script contains the Monte Carlo runs for the "reduced effort" control 
# law developed in mbe_demo.jl.
# 
# Navigate to the directory containing this file and run:
# 
# julia> include("mbe_demo_mc.jl")
# 

# To run the sims in parallel, we'll need to make some new workers.
if nworkers() == 1 # Add them iff we haven't already added some workers.
    addprocs(3) # Add 3 workers.
end

# The workers need to know about our models, so include them everywhere.
@everywhere include("models.jl")

module MBEDemoMC

#############
# Resources #
#############

# Use/import the modules we'll need.
using Eratosthenes   # The simulation engine (setup, simulate, mc)
using ..MBEModels    # This "uses" the MBEModels module defined in models.jl.
import HDF5          # For loading the log file
using Plots          # For plotting our results

# Create a directory for the log file.
if !isdir("out"); mkdir("out"); end

####################
# Monte Carlo Runs #
####################

# Build the scenario from the YAML file.
yaml_file = joinpath(@__DIR__, "underactuated.yaml")
scenario  = setup(yaml_file)

# There's no randomness in our sim by default. Let's randomize the initial
# attitude. The Body model that we're using (the default) has a parameter
# for the standard deviation of initial attitude perturbations. We'll
# overwrite that here and then run the MC with the updated scenario.
scenario.vehicles[1].body.constants.Δq = π/2 * ones(3)

# Choose a number of runs.
n_runs = 50

# Now tell Julia to run the scenarion n times. Each will have a different
# initial attitude.
mc(scenario, n_runs)

# Get the quaternions from the log file for every run.
t = Float64[] # The time steps will be the same for every run.
q = Float64[] # We'll store all of the quaternions here, side by side.
for k = 1:n_runs

    # Load from the log file.
    t, q_BI = HDF5.h5open("out/mc" * string(k) *".h5", "r") do logs
        (read(logs, "/smallsat/body/state/time"),
         read(logs, "/smallsat/body/state/data/q_BI"))
    end
    
    # Store the time and quaternions.
    t = squeeze(t, 1)
    q = vcat(q, q_BI)
    
end

# Choose a plotting backend.
plotlyjs()

# Show the quaternions from all runs.
println("Plotting all results.")
display(plot(t, q.',
    label  = "",
    xlabel = "Time (s)",
    ylabel = "Attitude Quaternions",
    title  = "Attitude from " * string(n_runs) * " Runs"))

# Next up: software-in-the-loop

end # module
