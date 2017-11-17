# mbe_demo_sitl.jl
#
# This script runs the C implementation of the control law in the simulation.

# Include our custom models.
include("models.jl")

# We work inside a module because it's more flexible than the main workspace.
module MBEDemoSITL

#############
# Resources #
#############

# Use/import the modules we'll need.
using Eratosthenes # The simulation engine (setup, simulate, mc)
using ..MBEModels  # The MBEModels module included above (and "above" this module)
import YAML        # For loading the YAML file directly
import HDF5        # For loading the log file
using Plots        # For plotting our results

# Create a directory for the log file.
if !isdir("out"); mkdir("out"); end

########################
# Software-in-the-Loop #
########################

# Specify the name of the YAML file that defines our scenario.
yaml_file = joinpath(@__DIR__, "underactuated.yaml")

# Load the YAML file and change the model name to use the SITL version.
# Also, change the log name.
yaml = YAML.load_file(yaml_file)
yaml["vehicles"][1]["computers"][1]["software"][1]["model"] = "ReducedEffortUnderactuatedControllerSITL"
yaml["sim"]["log_file"] = "out/underactuated-sitl.h5"

# The parameters are now stored under constants/parameters, so move them.
yaml["vehicles"][1]["computers"][1]["software"][1]["constants"] = Dict(
    "parameters"=>yaml["vehicles"][1]["computers"][1]["software"][1]["constants"]
);

# Set up the senario and run it.
scenario = setup(yaml)
scenario = simulate(scenario)

#########
# Plots #
#########

# Read some data from the logs.
t, q_BI, ω_BI_B = HDF5.h5open(scenario.sim.log_file, "r") do logs
    (squeeze(read(logs, "/smallsat/body/state/time"), 1),
     read(logs, "/smallsat/body/state/data/q_BI"),
     read(logs, "/smallsat/body/state/data/ω_BI_B"))
end

# Choose a friendly plotting package.
plotlyjs()

# Define each plot that we'll need.
display(plot(t, q_BI.',
             label  = ["q1" "q2" "q3" "q4"],
             xlabel = "Time (s)",
             ylabel = "Attitude Quaternion"))
display(plot(t, 180/π * ω_BI_B.',
             label  = ["ω1" "ω2" "ω3"],
             xlabel = "Time (s)",
             ylabel = "Rotation Rate (deg/s)"))

# Compare to the baseline.
(q_BI_baseline, ω_BI_B_baseline) = HDF5.h5open("out/underactuated.h5", "r") do logs
    (read(logs, "/smallsat/body/state/data/q_BI"),
     read(logs, "/smallsat/body/state/data/ω_BI_B"))
end
display(plot(t, q_BI.' - q_BI_baseline.',
    label  = ["q1" "q2" "q3" "q4"],
    xlabel = "Time (s)",
    ylabel = "Quaternion Differences",
    title  = "Differences in Attitude Quaternions"))
display(plot(t, 180/π * (ω_BI_B.' - ω_BI_B_baseline.'),
    label  = ["ω1" "ω2" "ω3"],
    xlabel = "Time (s)",
    ylabel = "Rate (deg/s)",
    title  = "Differences in Rotation Rate"))

end # module
