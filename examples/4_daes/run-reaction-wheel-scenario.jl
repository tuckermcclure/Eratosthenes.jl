# Make sure we're running from the examples directory.
cd(joinpath(@__DIR__(), ".."))

module Temp

using Eratosthenes # Bring in the top-level sim environment.
using Plots # For plotting (Pkg.add("Plots"); Pkg.add("PyPlot");)
import HDF5 # For loading the log file (Pkg.add("HDF5");)
using Printf

# Create the hierarchy of objects used by the sim.
scenario = setup("scenarios/reaction-wheel.yaml")

# Run it with text progress (e.g., from VS Code).
simulate(scenario) do k, n
    if (k-1) % 100 == 0
        @printf("Progress: % 5.1f%% done.\n", 100*(k-1)/(n-1))
    end
    return true
end

# Open the logs and plot some things.
if !isempty(scenario.sim.log_file)

    # Read some data from the logs. This do block takes care of
    # error-handling, so we can just read whatever we want and return it.
    t, q_BI, ω_BI_B, rw1, rw2, rw3 =
        HDF5.h5open(scenario.sim.log_file, "r") do logs
            (read(logs, "/smallsat1/body/state/time"),
             read(logs, "/smallsat1/body/state/data/q_BI"),
             read(logs, "/smallsat1/body/state/data/ω_BI_B"),
             read(logs, "/smallsat1/rw1/state/data/ω_RI_G"),
             read(logs, "/smallsat1/rw2/state/data/ω_RI_G"),
             read(logs, "/smallsat1/rw3/state/data/ω_RI_G"),
             )
        end

    # Scalars are logged as 1-by-n (don't know why). Convert to just n for
    # the sake of plotting.
    t = dropdims(t, dims=1)

    # Choose a friendly plotting package. PyPlot behaves nicely.
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

    display(plot(t, 30/π*[rw1[1,:] rw2[1,:] rw3[1,:]],
                 label  = ["rw1" "rw2" "rw3"],
                 xlabel = "Time (s)",
                 ylabel = "Wheel Rate (RPM)"))

end # plotting

end # Temp
