# Customized logging function for whole structures.
function add!(log::HDF5Logger.Log, slug::String, t::Real, data, n::Int)
    function add_structured!(log, slug, data, n)
        if isbits(data) || (isa(data, Array) && isbits(eltype(data)))
            HDF5Logger.add!(log, slug, data, n) # HDF5Loggeres only know scalars, vectors, and matrices.
        else
            for field in fieldnames(data)
                add_structured!(log, slug * "/" * string(field), getfield(data, field), n)
            end
        end
    end
    HDF5Logger.add!(log, slug * "time", t, n)
    add_structured!(log, slug * "data", data, n)
end

function log!(log::HDF5Logger.Log, slug::String, t::Real, data)
    function log_structured!(log, slug, data,)
        if isbits(data) || (isa(data, Array) && isbits(eltype(data)))
            HDF5Logger.log!(log, slug, data)
        else
            for field in fieldnames(data)
                log_structured!(log, slug * "/" * string(field), getfield(data, field))
            end
        end
    end
    HDF5Logger.log!(log, slug * "time", t)
    log_structured!(log, slug * "data", data)
end

"""
    simulate

This is the primary simulation function that begins with a set up scenario and
propagates until the end of time.

"""
function simulate(scenario::Scenario)

    # Create the log.
    # log = !isempty(scenario.sim.log_file) ? HDF5Logger.Log(scenario.sim.log_file) : nothing
    if !isempty(scenario.sim.log_file)
        println("Creating log $(scenario.sim.log_file).")
        log = HDF5Logger.Log(scenario.sim.log_file)
    else
        log = nothing
    end

    truth        = Dict{String,Any}() # This could contain vehicle truth as well as environment and actuator truth.
    measurements = Dict{String,Dict{String,Any}}() # "This is what I'll measure every sample."
    commands     = Dict{String,Dict{String,Any}}() # "This is how you tell me what to do."
    inputs       = Dict{String,Dict{String,Any}}() # "Anyone can write to these inputs. I will consume them."
    outputs      = Dict{String,Dict{String,Any}}() # "I will write to these outputs. Anyone can consume them."

    # Seed the global random number generator, which we'll only use to draw any
    # necessary seeds for other RNGs. When this is used as part of a Monte-Carlo
    # test, this global seed and all seeds below will be overridden. However,
    # we're not there yet; that's Milestone 3.
    srand(1)

    # Seed the components' RNGs.
    for vehicle in scenario.vehicles
        seed(vehicle.body.rand)
        for sensor   in vehicle.sensors;   seed(sensor.rand);   end;
        for actuator in vehicle.actuators; seed(actuator.rand); end;
    end

    # Make the time history.
    #
    # TODO: For now, the simulation always takes a step of dt, but it should
    # step only to the next discrete break (or dt, whichever is less).
    t = collect(0.:scenario.sim.dt:scenario.sim.t_end)
    if t[end] != scenario.sim.t_end # Stop at exactly the end time.
        t = [t senario.sim.t_end]
    end
    nt = length(t) # Total number of steps to take

    # If anything goes wrong, we'll want to tidy up here before rethrowing the
    # error.
    start_time = time_ns() # Start the timer.
    k = 1
    try

        ##################
        # Component Init #
        ##################

        # We'll initialize the state for each model and get the measurements,
        # commands, inputs, and outputs from the appropriate components.

        # Initialize each vehicle first.
        for vehicle in scenario.vehicles
            vehicle.body.state, truth[vehicle.name] =
                vehicle.body.init(t[1],
                                  vehicle.body.constants,
                                  vehicle.body.state,
                                  draw(vehicle.body.rand, :init))
        end

        # Now initialize each vehicle's subsystems
        for vehicle in scenario.vehicles
            measurements[vehicle.name] = Dict{String,Any}()
            commands[vehicle.name]     = Dict{String,Any}()
            inputs[vehicle.name]       = Dict{String,Any}()
            outputs[vehicle.name]      = Dict{String,Any}()
            for sensor in vehicle.sensors
                sensor.state,
                measurements[vehicle.name][sensor.name] =
                    sensor.init(t[1],
                                sensor.constants,
                                sensor.state,
                                draw(sensor.rand, :init),
                                truth[vehicle.name])
            end
            for software in vehicle.software
                software.state,
                inputs[vehicle.name][software.name],
                outputs[vehicle.name][software.name] =
                    software.init(t[1],
                                  software.constants,
                                  software.state)
            end
            for actuator in vehicle.actuators
                actuator.state,
                measurements[vehicle.name][actuator.name],
                commands[vehicle.name][actuator.name] =
                    actuator.init(t[1],
                                  actuator.constants,
                                  actuator.state,
                                  draw(actuator.rand, :init),
                                  truth[vehicle.name])
            end
        end

        ############
        # Sim Init #
        ############

        # Preallocate space for the streams will need in the logger.
        if isa(log, HDF5Logger.Log)

            println("Setting up log.")
            for vehicle in scenario.vehicles

                # Set up the streams for the sensors.
                for sensor in filter(s -> s.sense != nothing, vehicle.sensors)
                    slug = "/" * vehicle.name * "/measurements/" * sensor.name * "/"
                    num_samples = Int64(floor(scenario.sim.t_end / sensor.dt)) + 1
                    add!(log, slug, t[1], measurements[vehicle.name][sensor.name], num_samples)
                end

                # Set up the streams for the actuators.
                for actuator in filter(a -> a.sense != nothing, vehicle.actuators)
                    slug = "/" * vehicle.name * "/measurements/" * actuator.name * "/"
                    num_samples = Int64(floor(scenario.sim.t_end / actuator.dt)) + 1
                    add!(log, slug, t[1], measurements[vehicle.name][actuator.name], num_samples)
                end

                # Set up the streams for the vehicle's truth.
                slug = "/" * vehicle.name * "/truth/"
                add!(log, slug, t[1], truth[vehicle.name], nt)

            end

        end

        ############
        # Sim Loop #
        ############

        # TODO: Replace with a while and look for the next break time.
        for k = 1:nt

            # Update the progress bar.
            if scenario.sim.progress_fcn != nothing
                if scenario.sim.progress_fcn((k-1)/nt) == false
                    break;
                end
            end

            #############
            # Propagate #
            #############

            # Propagate from k-1 to k.
            if k > 1

                # Propagate the continuous-time stuff to the current time.
                propagate!(t[k-1], t[k], scenario, truth, commands)

            end

            # Log the truth.
            if isa(log, HDF5Logger.Log)
                for vehicle in scenario.vehicles
                    slug = "/" * vehicle.name * "/truth/"
                    log!(log, slug, t[k], truth[vehicle.name])
                end
            end

            # Update sensors, software, and actuators.
            for vehicle in scenario.vehicles

                ###########
                # Measure #
                ###########

                # Update any sensor whose t_next has come.
                for sensor in filter(s -> t[k] >= s.t_next, vehicle.sensors)

                    # Update the state and get the measurement.
                    if sensor.sense != nothing

                        measurements[vehicle.name][sensor.name] =
                            sensor.sense(t[k],
                                         sensor.constants,
                                         sensor.state,
                                         draw(sensor.rand, :sense),
                                         truth[vehicle.name],
                                         truth)

                        # Log it.
                        if isa(log, HDF5Logger.Log)
                            slug = "/" * vehicle.name * "/measurements/" * sensor.name * "/"
                            log!(log, slug, t[k], measurements[vehicle.name][sensor.name])
                        end

                    end

                    # Update the next sample time.
                    sensor.t_next += sensor.dt

                end # each sensor

                # Update any actuator whose t_next has come.
                for actuator in filter(a -> t[k] >= a.t_next, vehicle.actuators)

                    # Update the state and get the measurement.
                    if actuator.sense != nothing

                        measurements[vehicle.name][actuator.name] =
                            actuator.sense(t[k],
                                           actuator.constants,
                                           actuator.state,
                                           draw(actuator.rand, :sense),
                                           commands[vehicle.name][actuator.name],
                                           truth[vehicle.name],
                                           truth)

                        # Log it.
                        if isa(log, HDF5Logger.Log)
                            slug = "/" * vehicle.name * "/measurements/" * actuator.name
                            log!(log, slug, t[k], measurements[vehicle.name][actuator.name])
                        end
                    end

                    # Update the next sample time.
                    actuator.t_next += actuator.dt

                end # each actuator

                ############
                # Software #
                ############

                # Update any software whose t_next has come.
                for software in filter(s -> t[k] >= s.t_next, vehicle.software)

                    # Update the state and get the measurement.
                    # Make this "next inputs, outputs, and command"?
                    software.state,
                    outputs[vehicle.name][software.name], # This software can only write to its own outputs.
                    inputs[vehicle.name], # This software can write to anyone's inputs.
                    commands[vehicle.name] = # Commands are really just actuator inputs.
                        software.step(t[k],
                                      software.constants,
                                      software.state,
                                      measurements[vehicle.name], # Can read fromm all sensors
                                      inputs[vehicle.name][software.name], # This software can only read its own inputs.
                                      outputs[vehicle.name], # It can read anyone's outputs.
                                      inputs[vehicle.name], # It can write to anyone else's inputs (and should be considered write-only).
                                      commands[vehicle.name]) # It can write to anything on the command bus (and should be considered write-only).

                    # Update the next sample time.
                    software.t_next += software.dt

                    # # Log it.
                    # if isa(log, HDF5Logger.Log)
                    #     slug = "/" * vehicle.name * "/software/" * software.name
                    #     log!(log, slug, t[k], ???)
                    # end

                end # each software

            end

        end

    catch err

        # The sim stops upon receiving ctrl+c (ctrl+shift+c in Juno). That's not
        # really an error. All other errors are though.
        if isa(err, InterruptException)
            println("Stopping sim due to user interrupt.")
        else
            rethrow(err)
        end


    # Close the log file no matter what.
    finally

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

# We can simulate a whole scenario directly from the file name.
function simulate(file_name::String, context = current_module())
    return simulate(setup(Scenario(), file_name, context))
end

# Enable do-block syntax for the progress function.
# TODO: Remove the progress_fcn field from scenario?
function simulate(progress_fcn::Function, file_name::String, context = current_module())
    scenario = setup(Scenario(), file_name, context)
    scenario.sim.progress_fcn = progress_fcn
    return simulate(scenario)
end

# Enable do-block syntax for the progress function.
# TODO: Remove the progress_fcn field from scenario?
function simulate(progress_fcn::Function, scenario::Scenario)
    scenario.sim.progress_fcn = progress_fcn
    return simulate(scenario)
end

# Bring the states from t_{k-1} to t_k.
function propagate!(t_km1, t_k, scenario, truth, commands)

    # Time step (s)
    dt = t_k - t_km1

    #####################
    # Continuous States #
    #####################

    # Extract all of the states. (For now, only vehicle bodies have
    # continuous-time states.)
    states = [vehicle.body.state for vehicle in scenario.vehicles]

    # Run Runge-Kutta 4 until we get an integrator passed into here.
    Xd1 = continuous(t_km1,         states,               scenario, commands)
    Xd2 = continuous(t_km1 + 0.5dt, states + 0.5dt * Xd1, scenario, commands)
    Xd3 = continuous(t_km1 + 0.5dt, states + 0.5dt * Xd2, scenario, commands)
    Xd4 = continuous(t_km1 +    dt, states +    dt * Xd3, scenario, commands)
    states = states + dt/6. * Xd1 + dt/3. * Xd2 + dt/3. * Xd3 + dt/6. * Xd4

    # Put back all of the states.
    for (vehicle, state) in zip(scenario.vehicles, states)
        vehicle.body.state = state
    end

    # Get the vehicle truths.
    for vehicle in scenario.vehicles
        truth[vehicle.name] = vehicle.body.step(t_k, vehicle.body.constants, vehicle.body.state, nothing, scenario.planet)
    end

    ###################
    # Discrete States #
    ###################

    # Propagate the sensors and actuators to the current time.

    # Update sensors, software, and actuators.
    for vehicle in scenario.vehicles

        # Update any sensor whose t_next has come.
        for sensor in filter(s -> s.step != nothing && t_k >= s.t_next, vehicle.sensors)

            # Update the state and get the measurement.
            sensor.state =
                sensor.step(t_k,
                            sensor.constants,
                            sensor.state,
                            draw(sensor.rand, :step),
                            truth[vehicle.name],
                            truth)

            # Note: we don't update next hit time here (t_next), because we'll
            # do that when we "sense" for this sensor.

        end # each sensor

        # Update any actuator whose t_next has come.
        for actuator in filter(a -> a.step != nothing && t_k >= a.t_next, vehicle.actuators)

            # Update the state and get the measurement.
            actuator.state =
                actuator.step(t_k,
                              actuator.constants,
                              actuator.state,
                              draw(actuator.rand, :step),
                              commands[vehicle.name][actuator.name],
                              truth[vehicle.name],
                              truth)

            # Note: we don't update next hit time here (t_next), because we'll
            # do that when we "sense" for this actuator.

        end # each sensor

    end

end

function continuous(t, states, scenario, commands)

    # Assemble the momentary truth as a function of momentary state.
    # TODO: Can't we skip this for the first iteration?
    # TODO: Change 'step' to 'truth'.
    truth = Dict{String,Any}()
    for (vehicle, state) in zip(scenario.vehicles, states)
        truth[vehicle.name] = vehicle.body.step(t, vehicle.body.constants, state, nothing, scenario.planet)
    end

    # Create the set derivatives that we will write to below.
    derivs = deepcopy(states) # If I knew what 'states' was, I could just create an empty set.
    for (vehicle, state, k) in zip(scenario.vehicles, states, 1:length(states))

        # Get all of the actuator forces.
        forces = zeros(3, 2) # CentralForce with convert from LocalForce
        for actuator in vehicle.actuators
            forces += actuator.actuate(t,
                                       actuator.constants,
                                       actuator.state,
                                       draw(actuator.rand, :actuate),
                                       commands[vehicle.name][actuator.name],
                                       truth[vehicle.name],
                                       truth)
        end

        # Form the rate of change of the state (same type as state itself).
        derivs[k] = vehicle.body.derivatives(t,
                                             vehicle.body.constants,
                                             state,
                                             nothing, # TODO: Handle draws over continuous-time.
                                             scenario.planet,
                                             forces)

    end

    return derivs

end
