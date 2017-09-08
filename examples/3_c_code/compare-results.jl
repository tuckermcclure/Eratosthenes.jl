# Run JITL, SITL, and PITL first to generate their logs. Then run this script to
# compare them.

module Temp

using Plots
using HDF5

include(joinpath(Pkg.dir("Eratosthenes"), "src", "EratosthenesRotations.jl"))
using .EratosthenesRotations

# Nominal
t, q_BI = HDF5.h5open("out/basic-cube.h5", "r") do logs
    (read(logs, "/cube1/truth/time"),
     read(logs, "/cube1/truth/data/q_BI"))
end

# SITL
t_sitl, q_BI_sitl = HDF5.h5open("out/basic-cube-sitl.h5", "r") do logs
    (read(logs, "/cube1/truth/time"),
     read(logs, "/cube1/truth/data/q_BI"))
end

# PITL
t_pitl, q_BI_pitl = HDF5.h5open("out/basic-cube-pitl.h5", "r") do logs
    (read(logs, "/cube1/truth/time"),
     read(logs, "/cube1/truth/data/q_BI"))
end

# Scalars are logged as 1-by-n (don't know why). Convert to just n for
# the sake of plotting.
t      = squeeze(t, 1)
t_sitl = squeeze(t_sitl, 1)
t_pitl = squeeze(t_pitl, 1)

# Compare the literal values in the quaternion (not the underlying rotation);
# they should be identical.
Δq_sitl = q_BI - q_BI_sitl
display(maximum(Δq_sitl, 2))
Δq_pitl = q_BI - q_BI_pitl
display(maximum(Δq_pitl, 2))
Δq_pitl_wrt_sitl = q_BI_sitl - q_BI_pitl
display(maximum(Δq_pitl_wrt_sitl, 2))

# Ok, also compare the angular differences.
θ_sitl = map(k -> q2aa(qpos(qdiff(q_BI[:,k], q_BI_sitl[:,k])))[1], 1:size(q_BI, 2))
display(maximum(θ_sitl) * 180/pi)
θ_pitl = map(k -> q2aa(qpos(qdiff(q_BI[:,k], q_BI_pitl[:,k])))[1], 1:size(q_BI, 2))
display(maximum(θ_pitl) * 180/pi)
θ_pitl_wrt_sitl = map(k -> q2aa(qpos(qdiff(q_BI_sitl[:,k], q_BI_pitl[:,k])))[1], 1:size(q_BI_sitl, 2))
display(maximum(θ_pitl_wrt_sitl) * 180/pi)

# Show the divergence.
# pyplot()
plotlyjs()
# display(plot(t, Δq.',
#              label  = ["Δq1" "Δq2" "Δq3" "Δq4"],
#              ylabel = "Δq"))
display(plot(t, θ_pitl * (180./pi * 3600.),
             xlabel = "Time (s)",
             ylabel = "θ (arcseconds)"))

end
