reload("Eratosthenes")
sleep(0.01) # Give those warnings a second to get out of the way.

# Make sure we can find the things we want to use below. If these were all
# "real" packages, then we wouldn't need to do this.
if !in(pwd(), LOAD_PATH)
    println("Adding working directory to load path.");
    push!(LOAD_PATH, pwd())
end

module Work

using Plots
import HDF5 # For loading the log file
import Juno # For the progress bar

# Bring in the top-level sim environment as well as anything that the JSON
# file will need.
using Eratosthenes

# Create the hierarchy of objects used by the sim.
scenario = setup(Scenario(), "scenarios/basic-cube.yaml", Work)

# Run the sim. Outputs will be in the log file.
Juno.progress(name="Simulating...") do progress_bar
    scenario.sim.progress_fcn = fraction -> Juno.progress(progress_bar, fraction);
    simulate(scenario)
end

# Alternatives:
# simulate(scenario)
# simulate("inputs.json")

# Open the logs and plot some things.
if !isempty(scenario.sim.log_file)

    # Read some data from the logs. This do block takes care of
    # error-handling, so we can just read whatever we want and return it.
    # t_truth_sensor, q_BI, ω_BI_B =
    #     HDF5.h5open(scenario.sim.log_file, "r") do logs
    #         (read(logs, "/cube1/measurements/truth_sensor/time"),
    #          read(logs, "/cube1/measurements/truth_sensor/data/q_BI"),
    #          read(logs, "/cube1/measurements/truth_sensor/data/ω_BI_B"))
    #      end
    t, q_BI, ω_BI_B, t_st, q_BI_meas, t_gyro, ω_BI_B_meas =
        HDF5.h5open(scenario.sim.log_file, "r") do logs
            (read(logs, "/cube1/truth/time"),
             read(logs, "/cube1/truth/data/q_BI"),
             read(logs, "/cube1/truth/data/ω_BI_B"),
             read(logs, "/cube1/measurements/star_tracker/time"),
             read(logs, "/cube1/measurements/star_tracker/data"),
             read(logs, "/cube1/measurements/gyro/time"),
             read(logs, "/cube1/measurements/gyro/data"))
        end

    # Scalars are logged as 1-by-n (don't know why). Convert to just n for
    # the sake of plotting.
    t      = squeeze(t, 1)
    t_st   = squeeze(t_st, 1)
    t_gyro = squeeze(t_gyro, 1)

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

end # module Work

scenario = Work.scenario # Put the scenario into the Main module
display(scenario)        # and show it too.
