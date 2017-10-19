# TODO: gravity_j2, qEI, date2Y, all the Julian date stuff
# TODO: igrf12

# Point-mass gravity model
function point_mass_gravity(r, mu)
    return (-mu / norm(r)^3) * r
end
