# just-plot-it.jl

using Plots

# Choose a friendly plotting package. PyPlot behaves nicely.
# pyplot() # Stopped working after a recent update.
pyplot()

# Make data shaped like the data that had the problem.
t      = collect(0:3600) / 60.
q_BI   = randn(4, 3601)
ω_BI_B = randn(3, 3601)

# Define each plot that we'll need.
p1 = plot(t, q_BI',
            label  = ["q1" "q2" "q3" "q4"],
            ylabel = "Attitude Quaternion")
p2 = plot(t, 180/π * ω_BI_B',
            label  = ["ω1" "ω2" "ω3"],
            ylabel = "Rotation Rate (deg/s)")

# Display those plots as subplots using the default "2" layout.
display(plot(p1, p2,
             layout = 2,
             xlabel = "Time (s)"))

# If we comment out the above and only display one plot, it works the right way.
# display(p1)
