cd(dirname(@__FILE__))

module Temp

using Eratosthenes # Bring in the top-level sim environment.
using Printf

# Create the hierarchy of objects used by the sim.
scenario = setup("scenarios/reaction-wheel.yaml")

# Run it with text progress).
simulate(scenario) do k, n
    if (k-1) % 100 == 0
        @printf("Progress: % 5.1f%% done.\n", round(100*(k-1)/(n-1), digits=1))
    end
    return true
end

end # Temp
