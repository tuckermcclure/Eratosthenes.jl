include("getset.jl")

# Get all of the effects on all vehicles for the current state.
function get_effects(t, X, V, D, U, scenario)

    # Get all of the effects. Start with the vehicle body. Planets can consume
    # vehicle effects. Components can consume vehicle and planet effects.

    E = Dict{String, Dict{String, Tuple}}() # Instead of starting fresh, use the existing.
    c = 0 # TODO: Use something like component.id for this?
    for outer c = 1:length(scenario.vehicles)
        E[scenario.vehicles[c].name] = Dict{String, Tuple}()
        E[scenario.vehicles[c].name][scenario.vehicles[c].body.name] =
            scenario.vehicles[c].body.effects(
                t, scenario.vehicles[c].body.constants, X[c], nothing, V[c])
    end

    # Next, we'll get the effect of the environment on each body. While looping
    # over the bodies, we don't want the effects from the first body to be
    # visible to the second, so we'll need to make a copy here. We only need a
    # copy and not a deep copy, because the old effects won't be changed; we
    # simply need a dictionary that doesn't know about the new ones.
    effects_bus_copy = copy(E)

    # Get the planet's effects, given the body's effects.
    for vehicle in scenario.vehicles
        if scenario.environment.effects != nothing
            E[vehicle.name][scenario.environment.name] = scenario.environment.effects(t,
                                     scenario.environment.constants,
                                     scenario.environment.state, # TODO: Read from X once environments are allowed to have continuous-time states.
                                     draw(scenario.environment.rand, :effects), # TODO: This draw doesn't belong here. Same below.
                                     # TODO: Environments could have implicit variables.
                                     effects_bus_copy[vehicle.name],
                                     effects_bus_copy)
        end
    end

    # Get the actuators' effects, given the bodies' effects and planet's effects.
    effects_bus_copy = copy(E)
    for vehicle in scenario.vehicles

        # Get all of the component and computer board effects.
        for component in vehicle.components
            c += 1
            if component.effects != nothing
                E[vehicle.name][component.name] = component.effects(t, component.constants,
                                                                    X[c], D[c], V[c],
                                                                    U[vehicle.name][component.name],
                                                                    effects_bus_copy[vehicle.name], effects_bus_copy)
            end
        end
        for computer in vehicle.computers
            c += 1
            if computer.board.effects != nothing
                E[vehicle.name][computer.name] = computer.board.effects(t, computer.board.constants,
                                                                        X[c], D[c], V[c],
                                                                        U[vehicle.name][computer.name][computer.board.name],
                                                                        effects_bus_copy[vehicle.name], effects_bus_copy)
            end
        end

    end

    return E

end

# Calculate the Jacobian of the constraint function wrt the state using central finite differences.
function dgdX(t, X, V, D, U, scenario, α=1e-6) # Use a ! in the name? It modifies X, but should return it to its original.
    c  = zeros(scenario.dims.nv) # Constraints
    J  = zeros(scenario.dims.nv, scenario.dims.nxd) # Jacobian
    for k = 1:scenario.dims.nx
        z = scenario.dims.x_to_xd[k] # Next unused column of J
        if z != 0 # If this k corresponds to a continuous-time state...
            xo = getk(X, k) # Store the original value.
            setk!(X, k, xo + α) # Perturb it.
            stackem!(c, dae(t, X, V, D, U, scenario), 0) # Get constraints.
            J[:,z] = c[:] # Copy from the vector to the right place in the Jacobian. Do this with views instead?
            setk!(X, k, xo - α) # And now do the negative.
            stackem!(c, dae(t, X, V, D, U, scenario), 0)
            J[:,z] = (J[:,z] - c) / (2α)
            setk!(X, k, xo) # Return x to what it was.
        end
    end
    return J
end

# Calculate the Jacobian of the derivatives function wrt the implicit variables using central finite differences.
function dfdV(t, X, V, D, U, scenario, α=1e-6) # Use a ! in the name? It modifies V, but should return it to its original.
    xd = zeros(scenario.dims.nxd)
    J  = zeros(scenario.dims.nxd, scenario.dims.nv)
    for k = 1:scenario.dims.nv
        vo = getk(V, k) # Store the original value
        setk!(V, k, vo + α) # Perturb in the positive direction.
        stackem!(xd, ode(t, X, V, D, U, scenario), 0) # Replace with a version that stacks directly into J[:,k].
        J[:,k] = xd[:] # Copy from the vector to the right place in the Jacobian. Do this with views instead?
        setk!(V, k, vo - α) # And now the negative.
        stackem!(xd, ode(t, X, V, D, U, scenario), 0)
        J[:,k] = (J[:,k] - xd) / (2α)
        setk!(V, k, vo) # Return V to what it was.
    end
    return J
end

# Levenberg-Marquardt root solving for the implicity parameters, V, that satisfy the constraint functions.
function solve_for_V(t, X, V, D, U, scenario, Gx, λ=1e-6, tol=1e-12, k_max=1000)
    Xd = Vector{Any}() # This is just here for scope.
    xd = zeros(scenario.dims.nxd) # Preallocated space for "stacked" derivative
    k  = 0
    while true

        Xd = ode(t, X, V, D, U, scenario) # Get the big set derivatives.
        stackem!(xd, Xd, 0) # Create a stack of Float64s for it.
        cd = Gx * xd # Calculate the time-derivative of the constraints.
        if all(abs.(cd) .< tol); break; end # See if the constraint function is satisfied.
        if k == k_max; error("Could not satisfy the constraints functions after k iterations on the implicit variables."); end
        k += 1 # Ok, we're committing to this.

        # Calculate the Jacobian of the time-derivative of the constraints wrt the
        # implicit variables and use a Levenberg-Marquardt search for the implicit
        # parameters driving the time-derivative of the constraints to zero.
        J   =  Gx * dfdV(t, X, V, D, U, scenario)
        JtJ =  transpose(J) * J
        A   =  JtJ + λ * diagm(0 => diag(JtJ))
        σ   =  -(pinv(A) * (transpose(J) * cd)) # Use pinv in case of mischief.

        # Update each element of the implicit variables.
        for c = 1:length(σ); setk!(V, c, getk(V, c) + σ[c]); end # Could make a single updatek!(V, c, foo) function.

    end
    (V, Xd)
end

# Get the derivatives of the entire system when there are algebraic constraints.
function minor(t, X, V, D, U, scenario)

    # If there are algebraic constraints, solve for the implicit forces.
    if scenario.dims.nv > 0

        # Let g be the constraint function.
        #
        # If 0 = g(t, x), then 0 = d/dt g, and by the chain rule 0 = d/dx g * dx/dt.
        # We have a function for dx/dt given the implicit variables, v: dx/dt = f(t, x, v).
        # We can figure out the Jacobian, Gx = d/dx g, using finite differencing.
        # Then, we can solve for v s.t. Gx * f(t, x, v) = 0.
        #
        # Constraints and continuous equations will always be solved together. Should the contraints
        # be the second output of the `derivatives` function? No, that would make everyone have to
        # output a tuple. Plus, it's a toss up between contraints functions having duplicated work
        # from the derivatives function and the derivatives function being commonly "heavier" than
        # the constraints function (and being often unnecessary).

        # First, solve the constraints function for whatever part of V is observable.
        # V = solve_for_V_index_0(t, X, V, D, U, scenario) # Use SVD and if all S == 0, then there are no index 0 constraints.

        # Now differentiate the constraints function wrt the state.
        Gx = dgdX(t, X, V, D, U, scenario, 1e-9)

        # Now serach for implicit variables that put the state time derivative in the null
        # space of the constraint Jacobian. This solves for "index 1" constraints.
        V, Xd = solve_for_V(t, X, V, D, U, scenario, Gx)

    # Otherwise, just pass through whatever nothingness we're using for them.
    else
        Xd = ode(t, X, V, D, U, scenario)
    end

    (Xd, V)
end

# Get the vector of constraints.
function dae(t, X, V, D, U, scenario)

    # Get all of the effects corresponding to this state.
    E = get_effects(t, X, V, D, U, scenario)

    # Create the set of constraints that we will write to below.
    constraints = Vector{Float64}()

    # Get the constraints for each physical thing. We need to do this in the same
    # order as the states and draws arrays were built.
    c = 0
    for vehicle in scenario.vehicles
        c += 1
        if vehicle.body.constraints != nothing
            append!(constraints, vehicle.body.constraints(t, vehicle.body.constants,
                                                          X[c], D[c], V[c],
                                                          E[vehicle.name], E))
        end
    end

    for vehicle in scenario.vehicles
        for component in vehicle.components
            c += 1
            if component.constraints != nothing
                asdf = component.constraints(t, component.constants,
                    X[c], D[c], V[c],
                    U[vehicle.name][component.name],
                    E[vehicle.name], E)
                append!(constraints, asdf)
            end
        end
        for computer in vehicle.computers
            c += 1
            if computer.board.constraints != nothing
                append!(constraints, computer.board.constraints(t, computer.board.constants,
                                                                X[c], D[c], V[c],
                                                                U[vehicle.name][computer.name][computer.board.name],
                                                                E[vehicle.name], E))
            end
        end

    end

    return constraints

end

# Get the vector of derivatives.
function ode(t, X, V, D, U, scenario)

    # Get all of the effects corresponding to this state.
    E = get_effects(t, X, V, D, U, scenario)

    # Create the set of derivatives that we will write to below.
    derivs = Vector{Any}(undef, length(X)) # Would use eltype(X) but can't if we want to use nothing.
    fill!(derivs, nothing)

    # Get the derivatives for each physical thing. We need to do this in the same
    # order as the states and draws arrays were built. The derivative and the state
    # should be of the same type.
    c = 0
    for vehicle in scenario.vehicles
        c += 1
        if vehicle.body.derivatives != nothing
            derivs[c] = vehicle.body.derivatives(t, vehicle.body.constants,
                                                 X[c], D[c], V[c],
                                                 E[vehicle.name], E)
        end
    end

    for vehicle in scenario.vehicles
        for component in vehicle.components
            c += 1
            if component.derivatives != nothing
                derivs[c] = component.derivatives(t, component.constants,
                                                  X[c], D[c], V[c],
                                                  U[vehicle.name][component.name],
                                                  E[vehicle.name], E)
            end
        end
        for computer in vehicle.computers
            c += 1
            if computer.board.derivatives != nothing
                derivs[c] = computer.board.derivatives(t, computer.board.constants,
                                                       X[c], D[c], V[c],
                                                       U[vehicle.name][computer.name][computer.board.name],
                                                       E[vehicle.name], E)
            end
        end

    end

    return derivs

end
