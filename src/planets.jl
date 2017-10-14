mutable struct EarthConstants
    mu::Float64
    JD_0::Float64
    igrf_Y_0::Float64
    igrf::Any
    EarthConstants(; mu = 3.986004418e14, JD_0 = 0., igrf_Y_0 = 0., igrf = nothing) = new(mu, JD_0, igrf_Y_0, igrf)
end

function LowEarthOrbit()

    # Tell the sim that Earths have no state (anarchy?) and that they produce
    # Gravitys.
    # function init(t, constants, state, draws)
    #     return state
    # end

    # Output all of the effects on the vehicle.
    function effects(t, constants, state, draws, effects, effects_bus)
        body, found = find_effect(effects, BodyStateEffect(zeros(3), zeros(3), zeros(3), zeros(3)))
        @assert found > 0 "No body effects were found."
        return (Gravity(point_mass_gravity(body.r_be_I, constants.mu)),)
    end

    # Create the IdealActuator as a DynamicalModel.
    DynamicalModel("low_earth_orbit",
                   #init = init,
                   effects = effects,
                   constants = EarthConstants())

end
