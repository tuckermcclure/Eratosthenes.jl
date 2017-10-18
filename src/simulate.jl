include("logging.jl")

"""
    simulate

This is the primary simulation function that begins with a set up scenario and
propagates until the end of time.

"""
function simulate(progress_fcn::Union{Function,Void}, scenario::Scenario)

    # Create the log.
    if !isempty(scenario.sim.log_file)
        println("Creating log ", scenario.sim.log_file)
        log = HDF5Logger.Log(scenario.sim.log_file)
    else
        log = nothing
    end

    effects = Dict{String,Dict{String,Tuple}}() # "These are the effects that I generate."
    inputs  = Dict{String,Dict{String,Any}}()   # "Anyone can write to these inputs. I will consume them."
    outputs = Dict{String,Dict{String,Any}}()   # "I will write to these outputs. Anyone can consume them."

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

        ##################
        # Component Init #
        ##################

        # We'll initialize the state for each model and get the inputs and
        # outputs from the appropriate components.

        # Initialize the environment.
        if scenario.environment.init != nothing
            scenario.environment.init(t[1],
                                      scenario.environment.constants,
                                      scenario.environment.state,
                                      draw(scenario.environment.rand, :init))
        end

        # Initialize the vehicles.
        for vehicle in scenario.vehicles

            effects[vehicle.name] = Dict{String,Tuple}()

            # Initialize each vehicle body before doing components.
            if vehicle.body.init != nothing
                vehicle.body.state =
                    vehicle.body.init(t[1],
                                      vehicle.body.constants,
                                      vehicle.body.state,
                                      draw(vehicle.body.rand, :init))
            end
            if vehicle.body.effects != nothing
                effects[vehicle.name][vehicle.body.name] =
                    vehicle.body.effects(t[1],
                                         vehicle.body.constants,
                                         vehicle.body.state,
                                         draw(vehicle.body.rand, :effects))
            else
                effects[vehicle.name][vehicle.body.name] = () # TODO: Do I need this?
            end

            # Get the effects of the environment on the body.
            if scenario.environment.effects != nothing
                effects[vehicle.name][scenario.environment.name] =
                    scenario.environment.effects(t[1],
                                                 scenario.environment.constants,
                                                 scenario.environment.state,
                                                 draw(scenario.environment.rand, :effects),
                                                 effects[vehicle.name],
                                                 effects)
            else
                effects[vehicle.name][scenario.environment.name] = ()
            end

        end

        # Now initialize each vehicle's subsystems.
        for vehicle in scenario.vehicles

            inputs[vehicle.name]  = Dict{String,Any}()
            outputs[vehicle.name] = Dict{String,Any}()

            for component in vehicle.components

                if component.init != nothing
                    component.state, = component.init(
                        t[1],
                        component.constants,
                        component.state,
                        draw(component.rand, :init),
                        effects[vehicle.name],
                        effects)
                end

                if component.effects != nothing
                    effects[vehicle.name][component.name] = component.effects(
                        t[1],
                        component.constants,
                        component.state,
                        draw(component.rand, :effects),
                        component.inputs,
                        effects[vehicle.name],
                        effects) # TODO: Limit this to the body and environment effects.
                end

                inputs[vehicle.name][component.name]  = component.inputs
                outputs[vehicle.name][component.name] = component.outputs

            end

            for computer in vehicle.computers

                inputs[vehicle.name][computer.name]  = Dict{String,Any}()
                outputs[vehicle.name][computer.name] = Dict{String,Any}()

                if computer.board.init != nothing
                    computer.board.state = computer.board.init(
                        t[1],
                        computer.board.constants,
                        computer.board.state,
                        draw(computer.board.rand, :init),
                        effects[vehicle.name],
                        effects)
                end
                if computer.board.effects != nothing
                    effects[vehicle.name][computer.name][computer.board.name] = computer.board.effects(
                        t[1],
                        computer.board.constants,
                        computer.board.state,
                        draw(computer.board.rand, :effects),
                        computer.board.inputs,
                        effects[vehicle.name],
                        effects) # TODO: Limit this to the body and environment effects.
                end

                for software in computer.software

                    if software.init != nothing
                        software.state, = software.init(t[1],
                                                       software.constants,
                                                       software.state)
                    end

                    inputs[vehicle.name][computer.name][software.name]  = software.inputs
                    outputs[vehicle.name][computer.name][software.name] = software.outputs

                end

            end

        end

        # We now have the state of, inputs to, and outputs from all of pieces of
        # the simulation for the first sample. We also have all of the effects
        # resulting from the current state.
        #
        # I would like for the software to begin commanding immediately. Perhaps
        # the init function should also allow everything to update its outputs
        # and write to the inputs of others. How does this interact with the
        # idea that we can start the sim in a specific state, e.g. from half-way
        # through a previous run? TODO

        ############
        # Log Init #
        ############

        # Preallocate space for the streams we'll need in the logger.
        # TODO: Log on the first sample.
        if log != nothing

            println("Setting up log.")
            for vehicle in scenario.vehicles

                ## Set up the streams for the vehicle's truth.
                slug = "/" * vehicle.name * "/" * vehicle.body.name * "/state/"
                add!(log, slug, t[1], vehicle.body.state, nt)

                # Set up the streams for the components.
                for component in vehicle.components

                    # TODO: State

                    # TODO: Inputs

                    # Outputs
                    if component.outputs != nothing
                        slug = "/" * vehicle.name * "/" * component.name * "/outputs/"
                        num_samples = Int64(floor((scenario.sim.t_end - component.timing.t_start) / component.timing.dt)) + 1
                        add!(log, slug, t[1], outputs[vehicle.name][component.name], num_samples)
                    end

                end

                for computer in vehicle.computers

                    # TODO: State

                    # TODO: Inputs

                    # Outputs
                    if computer.board.outputs != nothing
                        slug = "/" * vehicle.name * "/" * computer.name * "/" * computer.board.name * "/outputs/"
                        num_samples = Int64(floor((scenario.sim.t_end - component.board.timing.t_start) / component.board.timing.dt)) + 1
                        add!(log, slug, t[1], outputs[vehicle.name][computer.name][computer.board.name], num_samples)
                    end

                    for software in computer.software

                        # TODO: State

                        # TODO: Inputs

                        # Outputs
                        if software.outputs != nothing
                            slug = "/" * vehicle.name * "/" * computer.name * "/" * software.name * "/outputs/"
                            num_samples = Int64(floor((scenario.sim.t_end - software.timing.t_start) / software.timing.dt)) + 1
                            add!(log, slug, t[1], outputs[vehicle.name][computer.name][software.name], num_samples)
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

            #############
            # Propagate #
            #############

            # Propagate from k-1 to k. We skip the first sample and just log for
            # k == 1.
            if k > 1

                # Time step (s)
                dt = t[k] - t[k-1]

                #####################
                # Continuous Update #
                #####################

                # Extract all of the continuous states. (For now, only vehicle bodies have
                # continuous-time states.)
                states = [vehicle.body.state for vehicle in scenario.vehicles]
                draws  = [draw(vehicle.body.rand, :derivatives) for vehicle in scenario.vehicles]
                # TODO: Handle draws for actuators.

                # Run Runge-Kutta 4 until we get an integrator passed into here.
                Xd1 = continuous(t[k-1],         states,               scenario, inputs, draws)
                Xd2 = continuous(t[k-1] + 0.5dt, states + 0.5dt * Xd1, scenario, inputs, draws)
                Xd3 = continuous(t[k-1] + 0.5dt, states + 0.5dt * Xd2, scenario, inputs, draws)
                Xd4 = continuous(t[k-1] +    dt, states +    dt * Xd3, scenario, inputs, draws)
                states = states + dt/6. * Xd1 + dt/3. * Xd2 + dt/3. * Xd3 + dt/6. * Xd4

                # Put back all of the states.
                for (vehicle, state) in zip(scenario.vehicles, states)
                    vehicle.body.state = state
                end

                # Get the effects on the current state. Note that this may
                # actually be different from the effects we'll need at the
                # beginning of the next propagation, because the discrete
                # updates below can change their states (and hense resulting
                # effects) instantly.
                effects = get_effects(t, states, scenario, inputs)

                ###################
                # Discrete Update #
                ###################

                # Update components and computer boards.
                for vehicle in scenario.vehicles

                    # TODO: Might bodies have discrete updates?

                    # Create an iterator over all of the effects for this vehicle for
                    # convenience and speed.
                    vehicle_effects = copy(effects[vehicle.name])

                    # Update any sensor whose t_next has come.
                    for component in vehicle.components
                        if component.update != nothing && t[k] >= component.timing.t_next - ϵ

                            # Update the state and get the measurement.
                            component.state,
                            outputs[vehicle.name][component.name] = component.update(
                                t[k],
                                component.constants,
                                component.state,
                                draw(component.rand, :update),
                                vehicle_effects,
                                inputs[vehicle.name][component.name],
                                effects)

                            # Update the next hit time.
                            component.timing.count += 1
                            component.timing.t_next = component.timing.dt * component.timing.count + component.timing.t_start

                            # TODO: Log.

                        end
                    end

                    # Update any sensor whose t_next has come.
                    for computer in vehicle.computers
                        if computer.board.update != nothing && t[k] >= computer.timing.t_next - ϵ

                            # Update the state and get the measurement.
                            computer.board.state,
                            outputs[vehicle.name][computer.name][computer.board.name] = computer.board.update(
                                t[k],
                                computer.board.constants,
                                computer.board.state,
                                draw(computer.board.rand, :update),
                                vehicle_effects,
                                inputs[vehicle.name][computer.name][computer.board.name],
                                effects)

                            # Update the next hit time.
                            computer.board.timing.count += 1
                            computer.board.timing.t_next = computer.board.timing.dt * computer.board.timing.count + computer.board.timing.t_start

                            # TODO: Log.

                        end
                    end

                end

            end

            ###################
            # Software Update #
            ###################

            # Update sensors, software, and actuators.
            for vehicle in scenario.vehicles

                vehicle_outputs_km1  = deepcopy(outputs[vehicle.name])
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
                            inputs[vehicle.name] = # Commands or software inputs for any other computer on this vehicle
                                software.update(t[k],
                                            software.constants,
                                            software.state,
                                            inputs[vehicle.name][computer.name][software.name], # Commands or software inputs from this vehicle
                                            vehicle_outputs_temp, # Includes measurements and other software outputs
                                            inputs[vehicle.name]) # To write to the inputs of other components

                            # Update the next sample time.
                            software.timing.count  += 1
                            software.timing.t_next = software.timing.dt * software.timing.count + software.timing.t_start

                            # Copy the updated outputs to the complete set of
                            # updated outputs that won't be used again until the
                            # next sample.
                            outputs[vehicle.name][computer.name][software.name] = vehicle_outputs_temp[computer.name][software.name]

                            # TODO: Log.

                        end
                    end # software

                    # Now that we're done with this component, return any of the
                    # modified outputs to their previous state.
                    vehicle_outputs_temp[computer.name] = deepcopy(vehicle_outputs_km1[computer.name])

                end # computer

                # The outputs bus has now been completely updated for this vehicle.

            end # vehicle

            # TODO: Or should we do all logging here?
            if log != nothing
                for vehicle in scenario.vehicles
                    slug = "/" * vehicle.name * "/" * vehicle.body.name * "/state/"
                    log!(log, slug, t[k], vehicle.body.state)
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

# Empty progress function
simulate(scenario::Scenario) = simulate(nothing, scenario)

# Simulation from file name
simulate(file_name::String, context = current_module()) = simulate(nothing, setup(Scenario(), file_name, context))

# Simulation from file name with progress function
simulate(progress_fcn::Function, file_name::String, context = current_module()) = simulate(progress_fcn, setup(Scenario(), file_name, context))

# Get all of the effects on all vehicles for the current state.
function get_effects(t, states, scenario, inputs)

    # Get all of the effects. Start with the vehicle body. Planets can consume
    # vehicle effects. Components can consume vehicle and planet effects.

    effects = Dict{String, Dict{String, Tuple}}() # Instead of starting fresh, use the existing.
    for k = 1:length(scenario.vehicles)
        effects[scenario.vehicles[k].name] = Dict{String, Tuple}()
        effects[scenario.vehicles[k].name][scenario.vehicles[k].body.name] =
            scenario.vehicles[k].body.effects(
                t, scenario.vehicles[k].body.constants, states[k], nothing)
    end

    # Next, we'll get the effect of the environment on each body. While looping
    # over the bodies, we don't want the effects from the first body to be
    # visible to the second, so we'll need to make a copy here. We only need a
    # copy and not a deep copy, because the old effects won't be changed; we
    # simply need a dictionary that doesn't know about the new ones.
    effects_bus_copy = copy(effects)

    # Get the planet's effects, given the body's effects.
    for vehicle in scenario.vehicles
        if scenario.environment.effects != nothing
            effects[vehicle.name][scenario.environment.name] = scenario.environment.effects(t,
                                     scenario.environment.constants,
                                     scenario.environment.state,
                                     draw(scenario.environment.rand, :effects), # TODO: This draw doesn't belong here. Same below.
                                     effects_bus_copy[vehicle.name],
                                     effects_bus_copy)
        end
    end

    # Get the actuators' effects, given the bodies' effects and planet's effects.
    effects_bus_copy = copy(effects)
    for vehicle in scenario.vehicles

        # Get all of the component and computer board effects.
        for component in vehicle.components
            if component.effects != nothing
                effects[vehicle.name][component.name] = component.effects(t,
                                           component.constants,
                                           component.state,
                                           draw(component.rand, :effects),
                                           inputs[vehicle.name][component.name],
                                           effects_bus_copy[vehicle.name],
                                           effects_bus_copy)
            end
        end
        for computer in vehicle.computers
            if computer.board.effects != nothing
                effects[vehicle.name][computer.name] = computer.board.effects(t,
                                           computer.board.constants,
                                           computer.board.state,
                                           draw(computer.board.rand, :effects),
                                           inputs[vehicle.name][computer.name][computer.board.name],
                                           effects_bus_copy[vehicle.name],
                                           effects_bus_copy)
            end
        end

    end

    return effects

end

function continuous(t, states, scenario, inputs, draws)

    effects = get_effects(t, states, scenario, inputs)

    # Create the set of derivatives that we will write to below.
    #
    # If I knew what 'states' was, I could just create an empty set. Also,
    # there's probably no advantage to creating this first. We should just
    # create an empty vector of eltype(states) and then push! the new states
    # into the vector.
    derivs = deepcopy(states)

    # Get the derivatives for each body.
    for (vehicle, k) in zip(scenario.vehicles, 1:length(states))

        # Form the rate of change of the state (same type as state itself).
        derivs[k] = vehicle.body.derivatives(t,
                                             vehicle.body.constants,
                                             states[k],
                                             draws[k],
                                             effects[vehicle.name],
                                             effects)

        # TODO: Get derivatives for actuators.

    end

    return derivs

end

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
