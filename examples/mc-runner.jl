# TODO:
# * Force seeds.
# * Add MC progress fcn.
# * Connect to needs_init.
# * Make log name.
# * Show logged results.

workspace()

@everywhere include("EratosthenesExampleSoftware.jl")

module Temp

using Eratosthenes # Bring in the top-level sim environment.

using ..EratosthenesExampleSoftware

scenario = setup("scenarios/basic-cube.yaml")
results = mc(scenario, 5)

display(results)

end
