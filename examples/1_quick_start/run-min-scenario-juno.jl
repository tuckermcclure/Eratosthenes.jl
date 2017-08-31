# Make sure we're running from the examples directory.
cd(joinpath(@__DIR__(), ".."))

using Eratosthenes # Bring in the top-level sim environment.
using Plots # For plotting (Pkg.add("Plots"); Pkg.add("PyPlot");)
import HDF5 # For loading the log file (Pkg.add("HDF5");)
import Juno # For the progress bar

# Create a Juno waitbar. This do-block syntax makes sure it gets "closed" even
# if there's an error.
scenario = Juno.progress(name="Simulating...") do progress_bar

    # Run the simulation. Anything inside this do-block will be run at the
    # beginning of every major time step. This is useful for updating a progress
    # bar or a plot.
    scenario = simulate("scenarios/min-scenario.yaml") do fraction
        Juno.progress(progress_bar, fraction) # Update Juno's progress bar.
        return true # Return false to end the sim.
    end

end

# Open the logs and plot some things.
if !isempty(scenario.sim.log_file)

    # Read some data from the logs. This do block takes care of
    # error-handling, so we can just read whatever we want and return it.
    t, q_BI, ω_BI_B = HDF5.h5open(scenario.sim.log_file, "r") do logs
        (read(logs, "/cube1/truth/time"),
         read(logs, "/cube1/truth/data/q_BI"),
         read(logs, "/cube1/truth/data/ω_BI_B"))
    end

    # Scalars are logged as 1-by-n (don't know why). Convert to just n for
    # the sake of plotting.
    t = squeeze(t, 1)

    # Choose a friendly plotting package. PyPlot behaves nicely.
    pyplot()

    # Define each plot that we'll need.
    p1 = plot(t, q_BI.',
              label  = ["q1" "q2" "q3" "q4"],
              ylabel = "Attitude Quaternion")
    p2 = plot(t, 180/π * ω_BI_B.',
              label  = ["ω1" "ω2" "ω3"],
              ylabel = "Rotation Rate (deg/s)")

    # Save the individual plots before showing them together.
    savefig(p1, "out/sim-results-q.png")
    savefig(p2, "out/sim-results-ω.png")

    # Display those plots as subplots using the default "2" layout. Put any
    # common descriptors here.
    display(plot(p1, p2,
                 layout = 2,
                 xlabel = "Time (s)"))

end # plotting
