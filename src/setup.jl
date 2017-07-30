# Create a function to convert a vector of vectors (e.g., list of list from YAML) to a matrix.
function Base.convert(::Type{Matrix{T}}, d::Vector{Vector{T}}) where {T}
    return hcat(d...)
end

# Create a function to turn a JSON file name into a dictionary and then load that dictionary.
function setup(model, file_name::String, context::Module = current_module())
    println("Loading scenario specifications from '$file_name' in $context.")
    # The `do` pattern here take care of closing the file if anything goes wrong.
    if endswith(lowercase(file_name), ".json")
        specs = open(file_name, "r") do inputs
            JSON.parse(inputs)
        end
    elseif endswith(lowercase(file_name), ".yaml")
        specs = YAML.load_file(file_name)
    else
        error("I don't know how to load that file type.")
    end
    display(specs)
    return setup(model, specs, context)
end

# setup fields matching keys with values from the dictionary.
function setup(model, specs::Dict, context::Module = current_module(), spacing::String = "")

    # If this requires that we put in a new model, do so now. Afterwards, we'll skip the "model" field.
    if haskey(specs, "model")

        println(spacing, "Making new model (", specs["model"], ").")

        try

            # Turn the model text into an expression.
            model_expr = parse(specs["model"])

            # Find the right place to evaluate this thing.
            if isa(model_expr, Symbol) && !isdefined(context, model_expr)
                mod = Eratosthenes
            else
                mod = context
            end

            # We evaluate the expression to give a function, which ought to construct the model.
            model_constructor = eval(mod, parse(specs["model"]))

            # Run the model constructor without arguments.
            model = model_constructor()

            println(spacing, "Created a default ", specs["model"], ".")

        catch err
            if isa(err, UndefVarError)
                println(spacing, "Model creation (", specs["model"], ") failed.")
            else
                rethrow(err)
            end
            return nothing
        end

    end

    # Um, how did this happen?
    if model == nothing
        error("The model contained nothing.")
    end

    # Let's now try to map each key in the dictionary to a field of the current model.
    println(spacing, "Looking through fields referenced in the Dict...")
    for (key, value) in specs

        # Skip descriptions and model specs.
        if contains(==, ["description", "model"], lowercase(key))
            continue
        end

        println(spacing, "Processing '$key'.")

        # Get the symbol for the field and make sure it exists.
        field = parse(key)
        if isdefined(model, field)

            # If the specs on the right are a dictionary, go deeper.
            if isa(value, Dict)

                println(spacing, "The '$key' field is a dictionary. Delving into it.")
                setfield!(model, field,
                setup(getfield(model, field), specs[key], context, spacing * "  "))

            # Otherwise, if it's a Vector, blow it away and replace it with what's in the specs.
            # A vector necessarily constructs everything from scratch, so they must
            # all specify which model to use.
            elseif isa(getfield(model, field), Vector) && !(eltype(getfield(model, field)) <: Number)

                println(spacing, "$field is a vector ($(typeof(getfield(model, field))))!")

                x = Vector{eltype(getfield(model, field))}(length(value))
                for k = 1:length(value)
                    try # We put the try here so that the whole assignment gets skipped.
                        x[k] = setup(nothing, value[k], context, spacing * "  ")
                    catch err
                        display(err)
                        println(spacing, "Attempting to carry on.")
                    end
                end
                setfield!(model, field, x)

                # Check for duplicated model names.
                model_names = [element.name for element in x]
                unique_model_names = unique(model_names)
                counts = [count(n -> n == model_name, model_names) for model_name in unique_model_names]
                if any(counts .> 1)
                    for model_name in unique_model_names[counts .> 1]
                        warn("Duplicate model name: " * model_name * ".")
                    end
                end

            # Otherwise, if this field is a function, make sure the right hand side evaluates to a function.
            elseif isa(getfield(model, field), Function) && isa(value, AbstractString)

                println("Trying to interpret the function: $value")
                converted = eval(context, parse(value))
                setfield!(model, field, converted)

            # Otherwise, it's presumed to be a "shallow" type.
            else

                # Set the field of the struct.
                println(spacing, "Overwriting $key ($(typeof(getfield(model, field)))) with $(typeof(value))).")
                converted = convert_it(context, getfield(model, field), value, spacing)
                setfield!(model, field, converted)

            end

        # If the field isn't part of the struct, throw an error.
        else
            # error("Sorry, but the field '$key' isn't part of the $(typeof(model)) type.")
            println(spacing, "Sorry, but the field '$key' isn't part of the $(typeof(model)) type. (Skipping.)")
        end

    end # for key, value...

    println(spacing, "Done.")
    return model

end

function convert_it(context, target, value, spacing)

    # Try to convert the value.
    target_type = typeof(target)
    source_type = typeof(value)

    # If they're already the same type, assign.
    if source_type == target_type

        println(spacing, "Types match. Assigning directly.")
        # converted_value = value
        converted_value = value

    # Otherwise, if there's a conversion from the source type to the target
    # type, use it.
    elseif method_exists(convert, Tuple{Type{target_type}, source_type})

        # Convert the value in the dictionary to the type contained in the
        # struct.
        println(spacing, "Converting $source_type to $target_type.")
        converted_value = convert(target_type, value)

    # If it's a string (and the target type isn't), see if we can evaluate it
    # into something useful.
    elseif isa(value, AbstractString)

        println("Evaluating $value.")
        evaluated = eval(context, parse(value))
        converted_value = convert_it(context, target, evaluated, spacing)

    # Otherwise, copy it and hope for the best.
    else
        println(spacing, "Copying")
        converted_value = deepcopy(value)
    end

    return converted_value

end
