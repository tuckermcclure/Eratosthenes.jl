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
                                     scenario.environment.state, # TODO: Read from states once environments are allowed to have continuous-time states.
                                     draw(scenario.environment.rand, :effects), # TODO: This draw doesn't belong here. Same below.
                                     # TODO: Environments can have implicit variables?
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

# Finite differences (central), where the state vector isn't actually a vector.
# This function takes advantage of the fact that the constraints and ode functions
# have the same interface.
function fdiffX(f::T, t, X, V, D, U, scenario, α=1e-6) where {T <: Function} # Use a ! in the name? It modifies x, but should return it to its original.
    fx = stackem(f(t, X, V, D, U, scenario)) # If n were passed in, this could just be full of 0.
    nf = length(fx) # This should be constant and could be passed in.
    nx = length(stackem(X)) # TODO: This is sloppy.
    J  = zeros(nf, nx)
    for k = 1:nx
        xo = getk(X, k) # Store the original value
        setk!(X, k, xo + α) # Perturb in the positive direction.
        stackem!(fx, f(t, X, V, D, U, scenario), 0)
        J[:,k] = fx[:] # Copy from the vector to the right place in the Jacobian. Do this with views instead?
        setk!(X, k, xo - α) # And now the negative.
        stackem!(fx, f(t, X, V, D, U, scenario), 0)
        J[:,k] = (J[:,k] - fx) / (2α)
        setk!(X, k, xo) # Return x to what it was.
    end
    return J
end

# Finite differences (central), where the state vector isn't actually a vector.
# This function takes advantage of the fact that the constraints and ode functions
# have the same interface.
function fdiffV(f::T, t, X, V, D, U, scenario, α=1e-6) where {T <: Function} # Use a ! in the name? It modifies x, but should return it to its original.
    Xd = f(t, X, V, D, U, scenario)
    fix_x_dot!(Xd) # TODO: Hack!
    fv = stackem(Xd) # If n were passed in, this could just be full of 0.
    nf = length(fv) # This should be constant and could be passed in.
    nv = length(stackem(V)) # TODO: This is sloppy.
    J  = zeros(nf, nv)
    for k = 1:nv
        vo = getk(V, k) # Store the original value
        setk!(V, k, vo + α) # Perturb in the positive direction.
        Xd = f(t, X, V, D, U, scenario)
        fix_x_dot!(Xd) # TODO: Hack!
        stackem!(fv, Xd, 0) # Replace with a version that stacks directly into J[:,k].
        J[:,k] = fv[:] # Copy from the vector to the right place in the Jacobian. Do this with views instead?
        setk!(V, k, vo - α) # And now the negative.
        Xd = f(t, X, V, D, U, scenario)
        fix_x_dot!(Xd) # TODO: Hack!
        stackem!(fv, Xd, 0)
        J[:,k] = (J[:,k] - fv) / (2α)
        setk!(V, k, vo) # Return V to what it was.
    end
    return J
end

# # Finite differences (central)
# function fdiff(f, x, α=1e-6)
#     J  = zeros(length(x), length(x))
#     for k = 1:length(x)
#         xo     = x[k] # Store the original value
#         x[k]   = xo + α # Perturb in the positive direction.
#         J[:,k] = f(x)
#         x[k]   = xo - α # And now the negative.
#         J[:,k] = (J[:,k] - f(x)) / (2α)
#         x[k]   = xo # Return x to what it was.
#     end
#     return J
# end

function fix_x_dot!(Xd)
    Xd[2] = zeros(8,1) # TODO: Hack!
    Xd[3] = Vector{Float64}()
    Xd[7] = Vector{Float64}()
end

# Levenberg-Marquardt root solving
function lmV(ode, t, X, V, D, U, scenario, Gx, λ=1e-6, tol=1e-12, k_max=1000)
    k = 0
    for k = 0:k_max-1
        Xd = ode(t, X, V, D, U, scenario)
        fix_x_dot!(Xd)
        fk = Gx * stackem(Xd)
        if all(abs.(fk) .< tol)
            break
        end
        J   =  Gx * fdiffV(ode, t, X, V, D, U, scenario)
        JtJ =  J.' * J
        A   =  JtJ + λ * diagm(diag(JtJ))
        σ   =  -(pinv(A) * (J.' * fk)) # Use pinv in case of mischief.
        for c = 1:length(σ)
            setk!(V, c, getk(V, c) + σ[c])
        end
    end
    Xd = ode(t, X, V, D, U, scenario) # TODO: Remove when hack is removed.
    (V, Xd, k)
end

# Get the derivatives of the entire system when there are algebraic constraints.
function minor(t, X, V, D, U, scenario)

    # If there are algebraic constraints, solve for the implicit forces.
    if !isempty(stackem(V)) # TODO: Sloppy!

        # Let g be the constraint function.
        # 
        # If 0 = g(t, x), then 0 = d/dt g, and by the chain rule 0 = d/dx g * dx/dt.
        # We have a function for dx/dt given the implicit variables, y: dx/dt = f(t, x, y).
        # We can figure out the Jacobian, Gx = d/dx g, using finite differencing.
        # Then, we can solve for y s.t. Gx * f(t, x, y) = 0.
        #
        # TODO: If the constraints are a function of the implicit variables, then this will fail,
        # so we've removed the implicit variables from that call for now. We could instead solve 
        # whatever constraints we can first, and then use that as a guess for the solution of the
        # derivatives vector.
        # 
        # Constraints and continuous equations will always be solved together. Should the contraints 
        # be the second output of the `derivatives` function?

        # First, solve the constraints function for whatever part of V is observable.
        # V = lm(V -> constraints(...), V, 1e-9)
        
        # Now differentiate the constraints function.
        Gx = fdiffX(dae, t, X, V, D, U, scenario, 1e-9)

        # Now serach for implicit variables that put the state time derivative in the null 
        # space of the constraint Jacobian.
        V, Xd = lmV(ode, t, X, V, D, U, scenario, Gx)

    # Otherwise, just pass through whatever nothingness we're using for them.
    else
        Xd = ode(t, X, V, D, U, scenario)
    end

    (Xd, V)

end

# Get the vector of constraints.
function dae(t, X, V, D, U, scenario)
    
    # Get all of the effects corresponding to this state.
    E = get_effects(t, X, V, D, U, scenario) # TODO: This could be reused.

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
    E = get_effects(t, X, V, D, U, scenario) # For DAEs, we already have this.

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
