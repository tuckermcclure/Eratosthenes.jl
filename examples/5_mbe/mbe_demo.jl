"""
mbe_demo.jl

This script contains the work towards a "reduced effort" control law for an
underactuated vehicle. In includes the initial simulation, the Monte-Carlo
runs, and TODO.

To run this file, start Julia, navigate to the directory containing this 
file, and just include it in your main workspace, like so:

```
$ julia
julia> cd(Pkg.dir("Eratosthenes") * "/examples/5_mbe")
julia> include("mbe_demo.jl")
```

"""
module MbeDemo # We work inside a module because it's more flexible than the main workspace.

# Bring in the simulation engine, including its main functions and types.
using Eratosthenes

# Bring in the custom models we're developing.
include("models.jl")
using .MBEModels

# Create a directory for the log file.
if !isdir("out"); mkdir("out"); end

# We'll want to load the log and plot the results in this script.
import HDF5
using Plots

# Run the scenario defined by our YAML file. The returned scenario will have all 
# of the final states.
scenario = simulate(joinpath(@__DIR__, "underactuated.yaml"))

# Open the logs and plot some things.
if !isempty(scenario.sim.log_file)
    
    # Read some data from the logs. This do block takes care of
    # error-handling, so we can just read whatever we want and return it.
    t, q_BI, ω_BI_B =
        HDF5.h5open(scenario.sim.log_file, "r") do logs
            (read(logs, "/smallsat/body/state/time"),
                read(logs, "/smallsat/body/state/data/q_BI"),
                read(logs, "/smallsat/body/state/data/ω_BI_B"),
                )
        end

    # Scalars are logged as 1-by-n. Convert to just n for the plot.
    t = squeeze(t, 1)

    # Choose a friendly plotting package.
    # pyplot()
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

end # plotting

# The plot looks fine, so we can now trying some Monte-Carlo runs.

# TODO

# Next up: software-in-the-loop.

# Then: hardware-in-the-loop.

end # module
