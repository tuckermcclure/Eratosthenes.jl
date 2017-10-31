include("logging.jl")
include("steps.jl")

# Empty progress function
simulate(scenario::Scenario) = simulate(nothing, scenario)

# Simulation from file name
simulate(file_name::String, context = current_module()) = simulate(nothing, setup(Scenario(), file_name, context))

# Simulation from file name with progress function
simulate(progress_fcn::Function, file_name::String, context = current_module()) = simulate(progress_fcn, setup(Scenario(), file_name, context))

"""
    simulate

This is the primary simulation function that begins with a set up scenario and
propagates until the end of time.

"""
function simulate(progress_fcn::Union{Function,Void}, scenario::Scenario, needs_init::Bool = true)

    # Create the log.
    if !isempty(scenario.sim.log_file)
        println("Creating log ", scenario.sim.log_file)
        log = HDF5Logger.Log(scenario.sim.log_file)
    else
        log = nothing
    end

    E = Dict{String,Dict{String,Tuple}}() # "These are the effects that I generate."
    U = Dict{String,Dict{String,Any}}()   # "Anyone can write to these inputs. I will consume them."
    Y = Dict{String,Dict{String,Any}}()   # "I will write to these outputs. Anyone can consume them."

    # Seed the global random number generator, which we'll only use to draw any
    # necessary seeds for other RNGs. When this is used as part of a Monte-Carlo
    # test, this global seed and all seeds below will be overridden. However,
    # we're not there yet; that's Milestone 3.
    srand(scenario.sim.seed)

    # Seed the components' RNGs.
    seed(scenario.environment.rand)
    for vehicle in scenario.vehicles
        seed(vehicle.body.rand)
        for component in vehicle.components; seed(component.rand); end;
        for computer in vehicle.computers; seed(computer.board.rand); end;
    end

    # Get the complete set of sample times.
    t, ϵ = calculate_time_steps(scenario)
    nt = length(t) # Total number of steps to take

    # Record the start time.
    start_time = time_ns()

    # If anything goes wrong, we'll want to tidy up here before rethrowing the
    # error.
    k = 1
    try

        ###########
        # Startup #
        ###########

        if scenario.environment.startup != nothing
            scenario.environment.startup(t[1], scenario.environment.constants, scenario.environment.state)
        end

        for vehicle in scenario.vehicles

            if vehicle.body.startup != nothing
                vehicle.body.startup(t[1], vehicle.body.constants, vehicle.body.state)
            end
            for component in vehicle.components
                if component.startup != nothing
                    component.startup(t[1], component.constants, component.state)
                end
            end
            for computer in vehicle.computers
                if computer.board.startup != nothing
                    computer.board.startup(t[1], computer.board.constants, computer.board.state)
                end
                for software in computer.software
                    if software.startup != nothing
                        software.startup(t[1], software.constants, software.state)
                    end
                end
            end

        end

        ########
        # Init #
        ########

        # We'll initialize the state for each model and get the inputs and
        # outputs from the appropriate components.

        # TODO: Much of the below looks redundant. Make little functions for this stuff.

        # Set up input and output buses.
        for vehicle in scenario.vehicles
            U[vehicle.name] = Dict{String,Any}()
            Y[vehicle.name] = Dict{String,Any}()
            U[vehicle.name][vehicle.body.name]  = vehicle.body.inputs
            Y[vehicle.name][vehicle.body.name] = vehicle.body.outputs
            for component in vehicle.components
                U[vehicle.name][component.name]  = component.inputs
                Y[vehicle.name][component.name] = component.outputs
            end
            for computer in vehicle.computers
                U[vehicle.name][computer.name] = Dict{String,Any}()
                Y[vehicle.name][computer.name] = Dict{String,Any}()
                U[vehicle.name][computer.name][computer.board.name] = computer.board.inputs
                Y[vehicle.name][computer.name][computer.board.name] = computer.board.outputs
                for software in computer.software
                    U[vehicle.name][computer.name][software.name] = software.inputs
                    Y[vehicle.name][computer.name][software.name] = software.outputs
                end
            end
        end

        # If the states and outputs are totally initialized already, we don't
        # need to run init.
        if needs_init

            # Initialize the environment.
            if scenario.environment.init != nothing
                scenario.environment.state =
                    scenario.environment.init(t[1],
                                              scenario.environment.constants,
                                              scenario.environment.state,
                                              draw(scenario.environment.rand, :init))
            end

            # Initialize the vehicles.
            for vehicle in scenario.vehicles

                E[vehicle.name] = Dict{String,Tuple}()

                # Initialize each vehicle body before doing components.
                if vehicle.body.init != nothing
                    vehicle.body.state =
                        vehicle.body.init(t[1],
                                          vehicle.body.constants,
                                          vehicle.body.state,
                                          draw(vehicle.body.rand, :init))
                end

                # Get the body's effects.
                if vehicle.body.effects != nothing
                    E[vehicle.name][vehicle.body.name] =
                        vehicle.body.effects(t[1],
                                             vehicle.body.constants,
                                             vehicle.body.state,
                                             draw(vehicle.body.rand, :effects),
                                             vehicle.body.implicit)
                else
                    #E[vehicle.name][vehicle.body.name] = () # TODO: Do we want to always have an entry, even if it were to produce no effects? It's a hash table, so it's not clear that this is problematic in terms of speed.
                end

                # Get the effects of the environment on the body.
                if scenario.environment.effects != nothing
                    E[vehicle.name][scenario.environment.name] =
                        scenario.environment.effects(t[1],
                                                     scenario.environment.constants,
                                                     scenario.environment.state,
                                                     draw(scenario.environment.rand, :effects),
                                                     E[vehicle.name],
                                                     E)
                else
                    #E[vehicle.name][scenario.environment.name] = ()
                end

                # TODO: Both Bodies and Environments could produce outputs.

            end

            # Now initialize each vehicle's subsystems.
            for vehicle in scenario.vehicles

                for component in vehicle.components

                    if component.init != nothing
                        component.state, Y[vehicle.name][component.name] = component.init(
                            t[1],
                            component.constants,
                            component.state,
                            draw(component.rand, :init),
                            U[vehicle.name][component.name],
                            E[vehicle.name],
                            E)
                    end

                    if component.effects != nothing
                        E[vehicle.name][component.name] = component.effects(
                            t[1],
                            component.constants,
                            component.state,
                            draw(component.rand, :effects),
                            component.implicit,
                            U[vehicle.name][component.name],
                            E[vehicle.name],
                            E) # TODO: Limit this to the body and environment effects.
                    end

                    # This counts as a "tick" of the system if this would have
                    # been an appropriate time for a step anyway.
                    if t[k] >= component.timing.t_next - ϵ
                        component.timing.count = 1
                        component.timing.t_next = component.timing.dt + component.timing.t_start
                    end

                end # for each component

                for computer in vehicle.computers

                    if computer.board.init != nothing
                        computer.board.state,
                        Y[vehicle.name][computer.name][computer.board.name] =
                            computer.board.init(
                                t[1],
                                computer.board.constants,
                                computer.board.state,
                                draw(computer.board.rand, :init),
                                U[vehicle.name][computer.name][computer.board.name],
                                E[vehicle.name],
                                E)
                    end

                    if computer.board.effects != nothing
                        E[vehicle.name][computer.name][computer.board.name] =
                            computer.board.effects(
                                t[1],
                                computer.board.constants,
                                computer.board.state,
                                draw(computer.board.rand, :effects),
                                computer.board.implicit,
                                U[vehicle.name][computer.name][computer.board.name],
                                E[vehicle.name],
                                E) # TODO: Limit this to the body and environment effects.
                    end

                    # This counts as a "tick" of the system if this would have
                    # been an appropriate time for a step anyway.
                    if t[k] >= computer.board.timing.t_next - ϵ
                        computer.board.timing.count = 1
                        computer.board.timing.t_next = computer.board.timing.dt + computer.board.timing.t_start
                    end

                    for software in computer.software

                        if software.init != nothing
                            software.state,
                            Y[vehicle.name][computer.name][software.name],
                            U[vehicle.name] =
                                software.init(
                                    t[1],
                                    software.constants,
                                    software.state,
                                    U[vehicle.name][computer.name][software.name],
                                    Y[vehicle.name],
                                    U[vehicle.name])
                        end

                        # This counts as a "tick" of the system if this would have
                        # been an appropriate time for a step anyway.
                        if t[k] >= software.timing.t_next - ϵ
                            software.timing.count = 1
                            software.timing.t_next = software.timing.dt + software.timing.t_start
                        end

                    end

                end # for each computer

            end # for each vehicle

        end # if needs_init

        # We now have the state of, inputs to, and outputs from all of pieces of
        # the simulation for the first sample. We also have all of the effects
        # resulting from the current state.

        ############
        # Log Init #
        ############

        # TODO: So that a sim can be recreated from a log, shouldn't we log the
        # sim parameters too? How do we log t_next/t_start?

        # Preallocate space for the streams we'll need in the logger.
        if log != nothing

            # TODO: Log environment.

            for vehicle in scenario.vehicles

                ## Set up the streams for the vehicle's truth.
                slug = "/" * vehicle.name * "/" * vehicle.body.name * "/state/"
                add!(log, slug, t[1], vehicle.body.state, nt)

                # Set up the streams for the components.
                for component in vehicle.components

                    # State
                    if component.state != nothing
                        slug = "/" * vehicle.name * "/" * component.name * "/state/"
                        if component.derivatives != nothing
                            num_samples = nt
                        else
                            num_samples = Int64(floor((scenario.sim.t_end - component.timing.t_start) / component.timing.dt)) + 1
                        end
                        add!(log, slug, t[1], component.state, num_samples)
                    end

                    # TODO: Inputs

                    # Outputs
                    if component.outputs != nothing
                        slug = "/" * vehicle.name * "/" * component.name * "/outputs/"
                        num_samples = Int64(floor((scenario.sim.t_end - component.timing.t_start) / component.timing.dt)) + 1
                        add!(log, slug, t[1], Y[vehicle.name][component.name], num_samples)
                    end

                end

                for computer in vehicle.computers

                    # TODO: State

                    # TODO: Inputs

                    # Outputs
                    if computer.board.outputs != nothing
                        slug = "/" * vehicle.name * "/" * computer.name * "/" * computer.board.name * "/outputs/"
                        num_samples = Int64(floor((scenario.sim.t_end - component.board.timing.t_start) / component.board.timing.dt)) + 1
                        add!(log, slug, t[1], Y[vehicle.name][computer.name][computer.board.name], num_samples)
                    end

                    for software in computer.software

                        # TODO: State

                        # TODO: Inputs

                        # Outputs
                        if software.outputs != nothing
                            slug = "/" * vehicle.name * "/" * computer.name * "/" * software.name * "/outputs/"
                            num_samples = Int64(floor((scenario.sim.t_end - software.timing.t_start) / software.timing.dt)) + 1
                            add!(log, slug, t[1], Y[vehicle.name][computer.name][software.name], num_samples)
                        end

                    end

                end

            end

        end

        ############
        # Sim Loop #
        ############

        # Loop over each index of the time array.
        for k = 1:nt

            # Update the progress bar.
            if progress_fcn != nothing
                if progress_fcn(k, nt) == false
                    break;
                end
            end

            # Put all of the continuous-time states, draws, and implicit variables 
            # into vectors.
            X = []
            D = []
            V = []
            # TODO: Add environments; they should be able to have continuous states.
            for vehicle in scenario.vehicles
                push!(X, vehicle.body.state)
                push!(D,  draw(vehicle.body.rand, :derivatives))
                push!(V, vehicle.body.implicit)
            end
            for vehicle in scenario.vehicles
                for component in vehicle.components
                    push!(X, component.state)
                    push!(D, draw(component.rand, :derivatives))
                    push!(V, component.implicit)
                end
                for computer in vehicle.computers
                    push!(X, computer.board.state)
                    push!(D, draw(computer.board.rand, :derivatives))
                    push!(V, computer.board.implicit)
                end
            end
            
            ##############
            # Dimensions #
            ##############

            # On the first sample, we discover some dimensions.
            if k == 1

                # We'll count up the number of states and the number of elements in the state 
                # derivative, as well as the mapping between them.
                nx  = 0
                nxd = 0
                x_to_xd = Vector{Int64}()

                # Make a dummy call to the derivatives.
                Xd = ode(t, X, V, D, U, scenario)

                # For each element of the state and its derivative...
                for c = 1:length(X)
                    nxc  = length(stackem(X[c]))  # Determine how many elements it has
                    nxdc = length(stackem(Xd[c])) # And the number of elements in the derivative.
                    if nxdc == 0 # If this state is discrete only, add a bunch of zero indices.
                        append!(x_to_xd, zeros(Int64, nxc))
                    elseif nxdc == nxc # Otherwise, map from state element to derivative elements.
                        append!(x_to_xd, nxd + (1:nxdc))
                    else # And if the numbers of those things don't match, it's a problem.
                        error("The number of derivatives don't match the number of states for the ", X[c], " type.")
                    end
                    nx  += nxc  # Count 'em up.
                    nxd += nxdc
                end

                # Store the results.
                scenario.dims = ScenarioDimensions(nt, nx, nxd, length(stackem(V)), x_to_xd)

            end
# display(X)
            #############
            # Propagate #
            #############
                
            # Propagate from k-1 to k. We skip the first sample and just log it.
            if k > 1
                
                #####################
                # Continuous Update #
                #####################

                # Use RK4 with a constraint solver for semi-explicity, index 1 DAE support.
                dt = t[k] - t[k-1] # Time step (s)
                Xd1, V = minor(t[k-1],         X,               V, D, U, scenario)
                Xd2, V = minor(t[k-1] + 0.5dt, X + 0.5dt * Xd1, V, D, U, scenario)
                Xd3, V = minor(t[k-1] + 0.5dt, X + 0.5dt * Xd2, V, D, U, scenario)
                Xd4, V = minor(t[k-1] +    dt, X +    dt * Xd3, V, D, U, scenario)
                X = X + dt/6. * Xd1 + dt/3. * Xd2 + dt/3. * Xd3 + dt/6. * Xd4

                # Put back all of the states.
                # TODO: Environments?
                # TODO: Why bother storing these this way?
                c = 0
                for vehicle in scenario.vehicles
                    c += 1
                    vehicle.body.state    = X[c]
                    vehicle.body.implicit = V[c]
                end
                for vehicle in scenario.vehicles
                    for component in vehicle.components
                        c += 1
                        component.state    = X[c]
                        component.implicit = V[c]
                    end
                    for computer in vehicle.computers
                        c += 1
                        computer.board.state    = X[c]
                        computer.board.implicit = V[c]
                    end
                end

                # Get the effects on the current state. Note that this may
                # actually be different from the effects we'll need at the
                # beginning of the next propagation, because the discrete
                # updates below can change their states (and hense resulting
                # effects) instantly.
                E = get_effects(t, X, V, D, U, scenario)

                ###################
                # Discrete Update #
                ###################

                # Update components and computer boards.
                for vehicle in scenario.vehicles

                    # TODO: Might bodies have discrete updates?

                    # Create an iterator over all of the effects for this vehicle for
                    # convenience and speed.
                    vehicle_effects = copy(E[vehicle.name])

                    # Update any sensor whose t_next has come.
                    for component in vehicle.components
                        if component.update != nothing && t[k] >= component.timing.t_next - ϵ

                            # Update the state and get the measurement.
                            component.state,
                            Y[vehicle.name][component.name] = component.update(
                                t[k],
                                component.constants,
                                component.state,
                                draw(component.rand, :update),
                                U[vehicle.name][component.name],
                                vehicle_effects,
                                E)

                            # Update the next hit time.
                            component.timing.count += 1
                            component.timing.t_next = component.timing.dt * component.timing.count + component.timing.t_start

                            # TODO: Log my inputs, resulting state, my outputs. Draws?
                            # Skip the state if there's a derivative function; it will be logged below.

                        end
                    end

                    # Update any sensor whose t_next has come.
                    for computer in vehicle.computers
                        if computer.board.update != nothing && t[k] >= computer.timing.t_next - ϵ

                            # Update the state and get the measurement.
                            computer.board.state,
                            Y[vehicle.name][computer.name][computer.board.name] = computer.board.update(
                                t[k],
                                computer.board.constants,
                                computer.board.state,
                                draw(computer.board.rand, :update),
                                U[vehicle.name][computer.name][computer.board.name],
                                vehicle_effects,
                                E)

                            # Update the next hit time.
                            computer.board.timing.count += 1
                            computer.board.timing.t_next = computer.board.timing.dt * computer.board.timing.count + computer.board.timing.t_start

                            # TODO: Log my inputs, resulting state, my outputs. Draws?
                            # Skip the state if there's a derivative function; it will be logged below.

                        end
                    end

                end

                ###################
                # Software Update #
                ###################

                # Update sensors, software, and actuators.
                for vehicle in scenario.vehicles

                    vehicle_outputs_km1  = deepcopy(Y[vehicle.name])
                    vehicle_outputs_temp = deepcopy(vehicle_outputs_km1)

                    # If computers were components and all components had software
                    # to run...
                    for computer in vehicle.computers # Filter on "active" components?

                        # Each computer can overwrite outputs for each software.

                        # Run each software, allowing subsequent processes to consume
                        # inputs/outputs from prior processes on this component.
                        for software in computer.software
                            if software.update != nothing && t[k] >= software.timing.t_next

                                # Any software of this computer can consume
                                # inputs/outputs from the prior components. They cannot
                                # see outputs from other components for this sample.

                                # Update the inputs bus and outputs for this software.
                                software.state,
                                vehicle_outputs_temp[computer.name][software.name], # This software's outputs
                                U[vehicle.name] = # Commands or software inputs for any other computer on this vehicle
                                    software.update(t[k],
                                                software.constants,
                                                software.state,
                                                U[vehicle.name][computer.name][software.name], # Commands or software inputs from this vehicle
                                                vehicle_outputs_temp, # Includes measurements and other software outputs
                                                U[vehicle.name]) # To write to the inputs of other components

                                # Update the next sample time.
                                software.timing.count  += 1
                                software.timing.t_next = software.timing.dt * software.timing.count + software.timing.t_start

                                # Copy the updated outputs to the complete set of
                                # updated outputs that won't be used again until the
                                # next sample.
                                Y[vehicle.name][computer.name][software.name] = vehicle_outputs_temp[computer.name][software.name]

                                # TODO: Log my inputs, resulting state, my outputs.

                            end
                        end # software

                        # Now that we're done with this component, return any of the
                        # modified outputs to their previous state.
                        vehicle_outputs_temp[computer.name] = deepcopy(vehicle_outputs_km1[computer.name])

                    end # computer

                    # The outputs bus has now been completely updated for this vehicle.

                end # for each vehicle

            end # k > 1

            ###########
            # Logging #
            ###########

            # Log the continuous states.
            if log != nothing
                for vehicle in scenario.vehicles
                    if vehicle.body.derivatives != nothing
                        slug = "/" * vehicle.name * "/" * vehicle.body.name * "/state/"
                        log!(log, slug, t[k], vehicle.body.state)
                    end
                    for component in vehicle.components
                        if component.derivatives != nothing
                            slug = "/" * vehicle.name * "/" * component.name * "/state/"
                            log!(log, slug, t[k], component.state)
                        end
                    end
                    for computer in vehicle.computers
                        if computer.board.derivatives != nothing
                            slug = "/" * vehicle.name * "/" * computer.name * "/" * computer.board.name * "/state/"
                            log!(log, slug, t[k], computer.board.state)
                        end
                    end
                end
            end

        end # sim loop

    catch err

        # TODO: Tidy up logs (get rid of unused preallocated space).

        # The sim stops upon receiving ctrl+c (ctrl+shift+c in Juno). That's not
        # really an error. All other errors really are errors though.
        if isa(err, InterruptException)
            println("Stopping sim due to user interrupt.")
            if k > 1; k -= 1; end # The last step completed was k-1.
        else
            rethrow(err)
        end

    # No matter what happens, we want to shut down when the sim has stopped
    # (either completed or errored out) so that we can tidy up resources.
    finally

        ############
        # Shutdown #
        ############

        # Shut down each vehicle first.
        for vehicle in scenario.vehicles

            if vehicle.body.shutdown != nothing
                vehicle.body.shutdown(t[k], vehicle.body.constants, vehicle.body.state)
            end
            for component in vehicle.components
                if component.shutdown != nothing
                    component.shutdown(t[k], component.constants, component.state)
                end
            end
            for computer in vehicle.computers
                if computer.board.shutdown != nothing
                    computer.board.shutdown(t[k], computer.board.constants, computer.board.state)
                end
                for software in computer.software
                    if software.shutdown != nothing
                        software.shutdown(t[k], software.constants, software.state)
                    end
                end
            end

        end

        # Close the log.
        if log != nothing
            close!(log)
        end

    end

    # Report how we did.
    run_time  = (time_ns() - start_time) * 1e-9
    println("Simulated ", t[k], " seconds in ", run_time, " seconds (", t[k]/run_time, "× real time).")

    return scenario
end

# Figure out every time step that we're going to need.
function calculate_time_steps(scenario)

    tf = scenario.sim.t_end
    dt = scenario.sim.dt

    # Timing slop. Discrete processes will occur when they're within this amount of
    # time from when they're supposed to happen.
    ϵ = eps(10 * scenario.sim.t_end)

    # Get all of the discrete time steps and start times.
    dts      = Vector{Float64}()
    t_starts = Vector{Float64}()
    for vehicle in scenario.vehicles
        # TODO: Bodies might have discrete steps.
        for component in Iterators.filter(c -> c.timing.dt != 0., vehicle.components)
            push!(dts, component.timing.dt)
            push!(t_starts, component.timing.t_start)
            component.timing.t_next = component.timing.t_start
        end
        for computer in vehicle.computers
            if computer.board.timing.dt != 0.
                push!(dts, computer.board.timing.dt)
                push!(t_starts, computer.board.timing.t_start)
                computer.board.timing.t_next = computer.board.timing.t_start
            end
            for software in Iterators.filter(s -> s.timing.dt != 0., computer.software)
                push!(dts, software.timing.dt)
                push!(t_starts, software.timing.t_start)
                software.timing.t_next = software.timing.t_start
            end
        end
    end

    # Make the time history.
    if !isempty(dts)

        t      = 0.
        ts     = Vector{Float64}() # array for time steps
        ndst   = copy(t_starts) # next discrete sample times
        counts = zeros(Float64, length(dts)) # how many times each discrete process has triggered

        # We can't need more than the sum of each process triggering on its own.
        # Create a counter so that, when it completes, we bail with an error.
        count = Int64(tf / dt + sum(tf ./ dts))

        # While we aren't already at the end of time...
        while t < tf

            # See if there's been a problem that's creating an infinite loop.
            count -= 1
            if count == 0
                error("Maximum number of steps exhausted; something went wrong when calculating time steps.")
            end

            # Find the next discrete sample time.
            tn = minimum(ndst)
            tn = min(tn, tf) # Don't step past the end of time.

            # See how many steps we'll need to take between now and then. Create
            # evenly-spaced steps between the points.
            Δt      = tn - t
            n_steps = ceil((Δt-ϵ)/dt)
            Δt_fit  = Δt / n_steps

            # Add on the evenly-spaced steps, up to the next discrete step.
            for k = 1:n_steps-1
                t += Δt_fit
                push!(ts, t) # Pushing each is better than vcating (no copies).
            end

            # Now add on the discrete step. (Doing this allows the step to occur
            # right at count * dt and avoids the accumulation of roundoff error over
            # time.
            t = tn
            push!(ts, t)

            # Update the times for the next steps of each discrete process.
            for k = 1:length(dts)
                if t > ndst[k] - ϵ
                    counts[k] += 1
                    ndst[k] = counts[k] * dts[k] + t_starts[k]
                end
            end

        end

    else

        # Otherwise, we have no discrete times. Step at the maximum rate.
        t = collect(0.:scenario.sim.dt:scenario.sim.t_end)
        if t[end] != scenario.sim.t_end # Stop at exactly the end time.
            push!(t, senario.sim.t_end)
        end

    end

    return (ts, ϵ, counts)

end
