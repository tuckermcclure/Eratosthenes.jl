using Eratosthenes # Bring in the top-level sim environment.
import HDF5 # For loading the log file (Pkg.add("HDF5");)

# If the out directory doesn't exist, make it.
if !isdir("out")
    mkdir("out")
end

# Run the sim on the simplest little scenario.
scenario = simulate(joinpath(Pkg.dir("Eratosthenes"), "examples", "scenarios", "min-scenario.yaml"))

# Open the logs and plot some things.
if !isempty(scenario.sim.log_file)

    # Read some data from the logs. This do block takes care of
    # error-handling, so we can just read whatever we want and return it.
    t, q_BI, ω_BI_B = HDF5.h5open(scenario.sim.log_file, "r") do logs
        (read(logs, "/cube1/body/state/time"),
         read(logs, "/cube1/body/state/data/q_BI"),
         read(logs, "/cube1/body/state/data/ω_BI_B"))
    end

    # Scalars are logged as 1-by-n (don't know why). Convert to just n for
    # the sake of plotting.
    t = dropdims(t, dims=1)

end # plotting
