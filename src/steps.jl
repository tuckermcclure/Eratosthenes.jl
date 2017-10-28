include("getset.jl")

# Get all of the effects on all vehicles for the current state.
function get_effects(t, X, V, D, U, scenario)

    # Get all of the effects. Start with the vehicle body. Planets can consume
    # vehicle effects. Components can consume vehicle and planet effects.

    E = Dict{String, Dict{String, Tuple}}() # Instead of starting fresh, use the existing.
    c = 0 # TODO: Use something like component.id for this?
    for c = 1:length(scenario.vehicles)
        E[scenario.vehicles[c].name] = Dict{String, Tuple}()
        E[scenario.vehicles[c].name][scenario.vehicles[c].body.name] =
            scenario.vehicles[c].body.effects(
                t, scenario.vehicles[c].body.constants, X[c], nothing)
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
                                     scenario.environment.state, # TODO: Read from states once environments are allowed to have continuous-time states.
                                     draw(scenario.environment.rand, :effects), # TODO: This draw doesn't belong here. Same below.
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
                E[vehicle.name][component.name] = component.effects(t,
                                           component.constants,
                                           X[c],
                                           D[c],
                                           U[vehicle.name][component.name],
                                           effects_bus_copy[vehicle.name],
                                           effects_bus_copy)
            end
        end
        for computer in vehicle.computers
            c += 1
            if computer.board.effects != nothing
                E[vehicle.name][computer.name] = computer.board.effects(t,
                                           computer.board.constants,
                                           X[c],
                                           D[c],
                                           U[vehicle.name][computer.name][computer.board.name],
                                           effects_bus_copy[vehicle.name],
                                           effects_bus_copy)
            end
        end

    end

    return E

end

# Finite differences (central), where the state vector isn't actually a vector.
function fdiffX(f, X, α=1e-6) # Use a ! in the name? It modifies x, but should return it to its original.
    fx = stackem(f(X)) # If n were passed in, this could just be full of 0.
    n  = length(fx) # This should be constant and could be passed in.
    J  = zeros(n, n)
    for k = 1:n
        xo = getk(X, k) # Store the original value
        setk!(X, k, xo + α) # Perturb in the positive direction.
        stackem!(fx, f(X), 0) # Replace with a version that stacks directly into J[:,k].
        J[:,k] = fx[:] # Copy from the vector to the right place in the Jacobian. Do this with views instead?
        setk!(X, k, xo - α) # And now the negative.
        stackem!(fx, f(X), 0)
        J[:,k] = (J[:,k] - fx) / (2α)
        setk!(X, k, xo) # Return x to what it was.
    end
    return J
end

# Finite differences (central)
function fdiff(f, x, α=1e-6)
    J  = zeros(length(x), length(x))
    for k = 1:length(x)
        xo     = x[k] # Store the original value
        x[k]   = xo + α # Perturb in the positive direction.
        J[:,k] = f(x)
        x[k]   = xo - α # And now the negative.
        J[:,k] = (J[:,k] - f(x)) / (2α)
        x[k]   = xo # Return x to what it was.
    end
    return J
end

# Levenberg-Marquardt root solving
function lm(G, f, β, λ=1e-6, tol=1e-6, k_max=1000)
    k = 0
    for k = 0:k_max-1
        fk = f(β)
        if all(abs.(fk) .< tol)
            break
        end
        J   =  fdiff(f, β)
        JtJ =  J.' * J
        A   =  JtJ + λ * diagm(diag(JtJ))
        σ   =  -(pinv(A) * (J.' * G * fk)) # Use pinv in case of mischief.
        β   += σ
    end
    (β, fk, k)
end

# Get the derivatives of the entire system when there are algebraic constraints.
function dae(t, X, V0, D, U, scenario)
    # Let g by the constraint function.
    # If 0 = g(t, x), then 0 = d/dt g, and by the chain rule 0 = d/dx g * dx/dt.
    # We have a function for dx/dt given the implicit variables, y: dx/dt = f(t, x, y)
    # We can figure out the Jacobian, Gx = d/dx g.
    # Then, we can solve for y s.t. Gx * f(t, x, y) = 0.
    #
    # TODO: If the constraints are a function of the implicit variables, then this will fail,
    # so we've removed the implicit variables from that call for now. We could instead solve 
    # whatever constraints we can first, and then use that as a guess for the solution of the
    # derivatives vector.
    # 
    # Constraints and continuous equations will always be solved together. Should the contraints 
    # be the second output of the `derivatives` function?
    Gx  = fdiffX(Xh -> constraints(t, Xh, V0, D, U, scenario), X, 1e-9)
    V   = lm(V -> Gx * stackem(continuous(t, X, V, D, U, scenario)), V0)
    Xd  = continuous(t, X, V, D, U, scenario) # This is already calculates above, but we'd need a custom LM to get it.
end

# Get the vector of derivatives.
function continuous(t, X, V, D, U, scenario)

    # Get all of the effects corresponding to this state.
    E = get_effects(t, X, V, D, U, scenario)

    # Create the set of derivatives that we will write to below.
    derivs = Vector{Any}(length(X)) # Would use eltype(X) but can't if we want to use nothing.
    fill!(derivs, nothing)

    # Get the derivatives for each physical thing. We need to do this in the same
    # order as the states and draws arrays were built. The derivative and the state
    # should be of the same type.
    c = 0
    for vehicle in scenario.vehicles
        c += 1
        if vehicle.body.derivatives != nothing
            derivs[c] = vehicle.body.derivatives(t,
                                                 vehicle.body.constants,
                                                 X[c],
                                                 D[c],
                                                 E[vehicle.name],
                                                 E)
        end
    end

    for vehicle in scenario.vehicles
        for component in vehicle.components
            c += 1
            if component.derivatives != nothing
                derivs[c] = component.derivatives(t,
                                                  component.constants,
                                                  X[c],
                                                  D[c],
                                                  U[vehicle.name][component.name],
                                                  E[vehicle.name],
                                                  E)
            end
        end
        for computer in vehicle.computers
            c += 1
            if computer.board.derivatives != nothing
                derivs[c] = computer.board.derivatives(t,
                                                       computer.board.constants,
                                                       X[c],
                                                       D[c],
                                                       U[vehicle.name][computer.name][computer.board.name],
                                                       E[vehicle.name],
                                                       E)
            end
        end

    end

    return derivs

end
