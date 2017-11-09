# TODO:
# * Force seeds.
# * Add MC progress fcn.
# * Connect to needs_init.
# * Make log name.
# * Show logged results.

# Monte Carlo runs
function mc(scenario::Scenario, n::Int)

    # Sets up each individual scenario (seed and log file) and runs it.
    function mchelper(scenario::Scenario, k::Int)
        println("Running MC variation ", k, " on worker ", myid())
        scenario.sim.seed = k
        scenario.sim.log_file = "out/mc" * string(k) * ".h5"
        simulate(scenario)
    end

    # Run the scenarios in parallel.
    pmap(k -> mchelper(scenario, k), 1:n)

end
