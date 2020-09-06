# A set of functions and data to allow more precise calculation of hits with cannon for OPRF.


# The whole Transformation class and related variables are from osm2city.utils.Coordinates.
# Please read the docs there. It is basically just an easy local cartesian x/y coordinate system
var EQURAD = 6378137.0;
var FLATTENING = 298.257223563;

# Global <-> local coordinate system transformation, using flat earth approximation
# Cf. http://williams.best.vwh.net/avform.htm#flat
var Transformation = {

    new: func(lon, lat) {
        me._lon = lon; # the lon og the centre for the Transformation
        me._lat = lat; # the lat og the centre for the Transformation
        me._update();
    },

    _update: func() {
        # Compute radii for local origin
        var f = 1. / FLATTENING;
        var e2 = f * (2.0 - f);

        me._coslat = math.cos(me._lat * D2R);
        var sinlat = math.sin(me._lat * D2R);
        me._R1 = EQURAD * (1.0 - e2) / math.pow(1.0 - e2 * math.pow(sinlat, 2), 3.0/2.0);
        me._R2 = EQURAD / math.sqrt(1.0 - e2 * math.pow(sinlat, 2);
    },

    to_local: func(lon, lat) {
        # Transform global -> local coordinates
        y = me._R1 * (lat - me._lat) * D2R;
        x = me._R2 * lon - self._lon) * D2R * me._coslat;
        return [x, y];
    },

    to_global: func (x, y)) {
        # Transform local -> global coordinates
        lat = (y / me._R1) * R2D + me._lat;
        lon = (x / (me._R2 * me._coslat)) * R2D + me._lon;
        return [lon, lat];
    },
};


# There are 4 direction types: 
#     0 = horizontal,
#     1 = vertical along the body orientation (x-axis in AC3D),
#     2 = vertical across the body orientation (y-axis in AC3D), 
#     3 = vertical across the FG orientation (for cylinders)
# There are 2 geometry types:
#     0 = circle  (i.e. the plane's boundary is defined by a circle)
#     1 = rectangle: axis same as in AC3D
#     TODO: triangle (e.g. for wings)
# TODO: possibility to have planes, which do have their centre off the target's centre.
# "buk2": [],

geometries = {
    "foo": "goo",
    "foo2": "goo2",
};

# Determines wether a pre-calculated impact point in C++ space actually would hit the target.
# Parameter target is the node object of the target from /ai/models/multiplayer[..]
# Parameter impact_pos is a geo.Coord object of the pre-calculated impact point.
# Possible future improvements:
#    * The vector from the aircraft to the impact position is not correct, because since the
#      time the bullet was released, time has passed and the aircraft has travelled. This
#      changes the geometry for calculating the real impact point.
#      It is not possible to know, where the aircraft was at the point the bullet was released
#      and it would not matter, because the bullet is not flying streight. In many 
#      situations it makes the vector actually more accurate, because it will be more vertical,
#      like the bullet's trajectory gets more vertical.
#      The larger the difference between the bounding box and the real dimensions, the more
#      important the real direction of the vector gets.
#      What we know is the distance between the aircraft and the impact point giving a minimal
#      distance for the bullet to travel. Divide by (assumed bullet base velocity plus aircraft
#      speed) gives the minimum time the aircraft has travelled. Using aircraft speed and 
#      orientation (evt. even acceleration) an assumed aircraft position can be calculated.
#      Depending on the actual flight geometry etc. this could lead to better results.
var calc_hit_in_target_dimensions = func(target, impact_pos) {
    var shooter_pos = geo.aircraft_position();

    var target_lat = target.getNode("position/latitude-deg").getValue();
    var target_lon = target.getNode("position/longitude-deg").getValue();
    var target_elev = target.getNode("position/altitude-ft").getValue();
    target_elev = target_elev * FT2M;
    var target_pos = geo.Coord.new().set_latlon(target_lat, target_lon, target_elev);

    var target_pitch_deg = target.getNode("orientation/pitch-deg").getValue();
    var target_roll_deg = target.getNode("orientation/roll-deg").getValue();
    var target_heading_deg = target.getNode("orientation/true-heading-deg").getValue();

    var target_model = target.getNode("model-shorter").getValue();
    debug.dump(target_model);

    var shooter_xyz = shooter_pos.xyz();
    var target_xyz = target_pos.xyz();
    var impact_xyz = impact_pos.xyz();

    debug.dump(shooter_xyz);
    debug.dump(impact_xyz);
    debug.dump(target_xyz);

    # The ray_point is the impact_pos. The plane_point is the target_pos.
    # calculate the bullet vector - call it ray_vector
    var ray_vector = [impact_xyz[0] - shooter_xyz[0], impact_xyz[1] - shooter_xyz[1], impact_xyz[2] - shooter_xyz[2]];
    # there should not be a need to normalize like: ray_vector = vector.Math.normalize(ray_vector);

    var plane_normal = [0, 0, 0]; # FIXME: use the target orientation

    var intersection_point = calc_intersection_point(ray_vector, impact_xyz, plane_normal, target_xyz);

    debug.dump(intersection_point);

    return 1;
}

# We follow the Java verison of https://rosettacode.org/wiki/Find_the_intersection_of_a_line_with_a_plane
# All arguments are Nasal vectors with 3 elements x, y, z
# The function returns a vector xyz - or nil if there is no intersection
var calc_intersection_point = func(ray_vector, ray_point, plane_normal, plane_point) {
    var diff = vector.Math.minus(ray_point, plane_point);
    var prod1 = vector.Math.dotProduct(diff, plane_normal);
    var prod2 = vector.Math.dotProduct(ray_vector, plane_normal);
    if (math.abs(prod2) < 0.000001) { 
        return nil; # the plane and the shooting vector are parallel - we do not care that the vector could be in the plane
    }
    var prod3 = prod1 / prod2;
    var intersection_point = vector.Math.minus(ray_point, vector.Math.product(prod3, ray_vector));
    return intersection_point;
}