# mbe_demo.jl
#
# This script contains the work towards a "reduced effort" control law for an
# underactuated vehicle. In includes the initial simulation, the Monte-Carlo
# runs, and TODO.
#
# To run this file, start Julia, navigate to the directory containing this
# file, and just include it in your main workspace, like so:
#
# ```
# $ julia
# julia> cd(Pkg.dir("Eratosthenes") * "/examples/5_mbe")
# julia> include("mbe_demo.jl")
# ```

# Include our custom models.
include("models.jl")

# We work inside a module because it's more flexible than the main workspace.
module MBEDemo

#############
# Resources #
#############

# Use/import the modules we'll need.
using Eratosthenes # The simulation engine (setup, simulate, mc)
using ..MBEModels  # The MBEModels module included above (and "above" this module)
import HDF5        # For loading the log file
using Plots        # For plotting our results
using LinearAlgebra

# Create a directory for the log file.
if !isdir("out"); mkdir("out"); end

##############
# Simulation #
##############

# Specify the name of the YAML file that defines our scenario.
yaml_file = joinpath(@__DIR__, "underactuated.yaml")

# Run the scenario defined by our YAML file. The returned scenario will
# have all of the final states.
scenario = simulate(yaml_file, @__MODULE__)

#########
# Plots #
#########

# Read some data from the logs. This do block takes care of
# error-handling, so we can just read whatever we want and return it.
t, q_BI, ω_BI_B = HDF5.h5open(scenario.sim.log_file, "r") do logs
    (read(logs, "/smallsat/body/state/time"),
     read(logs, "/smallsat/body/state/data/q_BI"),
     read(logs, "/smallsat/body/state/data/ω_BI_B"))
end

# Scalars are logged as 1-by-n. Convert to just n for the plot.
t = dropdims(t, dims=1)

# Choose a friendly plotting package.
pyplot()
# plotlyjs()

# Define each plot that we'll need.
display(plot(t, transpose(q_BI),
             label  = ["q1" "q2" "q3" "q4"],
             xlabel = "Time (s)",
             ylabel = "Attitude Quaternion"))
display(plot(t, 180/π * transpose(ω_BI_B),
             label  = ["ω1" "ω2" "ω3"],
             xlabel = "Time (s)",
             ylabel = "Rotation Rate (deg/s)"))

# The plot looks fine, so we can now try some Monte-Carlo runs.
# See mbe_demo_mc.jl.

end # module
