# A set of functions and data to allow more precise calculation of hits with cannon for OPRF.


# Determines wether a pre-calculated impact point in C++ space actually would hit the target.
# Parameter target is the node object of the target from /ai/models/multiplayer[..]
# Parameter impact_pos is a geo.Coord object of the pre-calculated impact point.
var calc_hit_in_target_dimensions = func(target, impact_pos) {
    var shooter_pos = geo.aircraft_position();

    var target_lat = target.getNode("position/latitude-deg").getValue();
    var target_lon = target.getNode("position/longitude-deg").getValue();
    var target_elev = target.getNode("position/altitude-ft").getValue();
    target_elev = target_elev * FT2M;

    var target_pos = geo.Coord.new().set_latlon(target_lat, target_lon, target_elev);

    debug.dump(shooter_pos);
    debug.dump(impact_pos);
    debug.dump(target_pos);

    return 1;
}