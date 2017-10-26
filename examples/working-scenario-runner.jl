# Make sure we're running from the examples directory.
# cd(@__DIR__())

reload("Eratosthenes")
sleep(0.1)

module Temp

# using BenchmarkTools
using Eratosthenes # Bring in the top-level sim environment.
using Plots # For plotting (Pkg.add("Plots"); Pkg.add("PyPlot");)
import HDF5 # For loading the log file (Pkg.add("HDF5");)
# import Juno # For the progress bar

# Include a module that's not part of Eratosthenes itself. Anything we "use"
# here will be visible during the setup portion of the scenario. So we can
# "setup" things that Eratosthenes doesn't even explicitly know about it!
# include("EratosthenesExampleSoftware.jl")
# using .EratosthenesExampleSoftware

# Create the hierarchy of objects used by the sim.
scenario = setup("scenarios/reaction-wheel.yaml")
# scenario.sim.log_file = ""

# # Just run it.
# simulate(scenario)

# # Profile it.
# scenario.sim.log_file = ""
# @profile simulate(scenario)
# open("profiler.txt", "w") do s
#     Profile.print(IOContext(s, :displaysize => (24, 500)))
# end
# Profile.clear()

# Run it with text progress (e.g., from VS Code).
simulate(scenario) do k, n
    if (k-1) % 100 == 0
        @printf("Progress: % 5.1f%% done.\n", round(100*(k-1)/(n-1), 1))
    end
    return true
end

# Create a Juno waitbar. This do-block syntax makes sure it gets "closed" even
# if there's an error.
# Juno.progress(name="Simulating...") do progress_bar
#
#     # Run the simulation. Anything inside this do-block will be run at the
#     # beginning of every major time step. This is useful for updating a progress
#     # bar or a plot.
#     simulate(scenario) do k, n
#         if k % 30 == 0
#             Juno.progress(progress_bar, (k-1)/n) # Update Juno's progress bar.
#         end
#         return true # Return false to end the sim.
#     end
#
# end

# Open the logs and plot some things.
if !isempty(scenario.sim.log_file)

    # Read some data from the logs. This do block takes care of
    # error-handling, so we can just read whatever we want and return it.
    t, q_BI, ω_BI_B, rw1, rw2, rw3 = # , t_st, q_BI_meas, t_gyro, ω_BI_B_meas
        HDF5.h5open(scenario.sim.log_file, "r") do logs
            (read(logs, "/cube1/body/state/time"),
             read(logs, "/cube1/body/state/data/q_BI"),
             read(logs, "/cube1/body/state/data/ω_BI_B"),
             read(logs, "/cube1/rw1/state/data"),
             read(logs, "/cube1/rw2/state/data"),
             read(logs, "/cube1/rw3/state/data"),
             #read(logs, "/cube1/star_tracker/outputs/time"),
             #read(logs, "/cube1/star_tracker/outputs/data"),
             #read(logs, "/cube1/gyro/outputs/time"),
             #read(logs, "/cube1/gyro/outputs/data")
             )
        end

    # Scalars are logged as 1-by-n (don't know why). Convert to just n for
    # the sake of plotting.
    t      = squeeze(t, 1)
    #t_st   = squeeze(t_st, 1)
    #t_gyro = squeeze(t_gyro, 1)

    # Choose a friendly plotting package. PyPlot behaves nicely.
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
    display(plot(t, 30/π*[rw1; rw2; rw3].',
                 label  = ["rw1" "rw2" "rw3"],
                 xlabel = "Time (s)",
                 ylabel = "Wheel Rate (RPM)"))
    display(plot(t, 30/π*squeeze(rw1,1),
                 xlabel = "Time (s)",
                 ylabel = "RW1 Rate (RPM)"))
    display(plot(t, 30/π*squeeze(rw2,1),
                 xlabel = "Time (s)",
                 ylabel = "RW2 Rate (RPM)"))
    display(plot(t, 30/π*squeeze(rw3,1),
                 xlabel = "Time (s)",
                 ylabel = "RW3 Rate (RPM)"))

    # Save the individual plots before showing them together.
    # savefig(p1, "out/sim-results-q.png")
    # savefig(p2, "out/sim-results-ω.png")

    # Display those plots as subplots using the default "2" layout. Put any
    # common descriptors here.
    # display(plot(p1, p2, p3,
    #              layout = 3,
    #              xlabel = "Time (s)"))

end # plotting

end # Temp
