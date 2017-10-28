workspace()

@everywhere include("EratosthenesExampleSoftware.jl")

module Temp

using Eratosthenes # Bring in the top-level sim environment.

using ..EratosthenesExampleSoftware

scenario = setup("scenarios/basic-cube.yaml")
results = mc(scenario, 5)

display(results)

end
