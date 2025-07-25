#pragma once

#include <AP_Math/AP_Math.h>

#define LOCATION_ALT_MAX_M  83000   // maximum altitude (in meters) that can be fit into Location structure's alt field

class Location
{
public:

    uint8_t relative_alt : 1;           // 1 if altitude is relative to home
    uint8_t loiter_ccw   : 1;           // 0 if clockwise, 1 if counter clockwise
    uint8_t terrain_alt  : 1;           // this altitude is above terrain
    uint8_t origin_alt   : 1;           // this altitude is above ekf origin
    uint8_t loiter_xtrack : 1;          // 0 to crosstrack from center of waypoint, 1 to crosstrack from tangent exit location

    // note that mission storage only stores 24 bits of altitude (~ +/- 83km)
    int32_t alt; // in cm
    int32_t lat; // in 1E7 degrees
    int32_t lng; // in 1E7 degrees

    /// enumeration of possible altitude types
    enum class AltFrame {
        ABSOLUTE = 0,
        ABOVE_HOME = 1,
        ABOVE_ORIGIN = 2,
        ABOVE_TERRAIN = 3
    };

    /// constructors
    Location() { zero(); }
    Location(int32_t latitude, int32_t longitude, int32_t alt_in_cm, AltFrame frame);
    Location(const Vector3f &ekf_offset_neu, AltFrame frame);
    Location(const Vector3d &ekf_offset_neu, AltFrame frame);

    // set altitude
    void set_alt_cm(int32_t alt_cm, AltFrame frame);
    // set_alt_m - set altitude in metres
    void set_alt_m(float alt_m, AltFrame frame) {
        set_alt_cm(alt_m*100, frame);
    }

    // get altitude (in cm) in the desired frame
    // does not modify ret_alt_cm unless true is returned
    // returns false on failure to get altitude in the desired frame which can only happen if the original frame or desired frame is:
    // - above-terrain and the terrain database can't supply terrain height amsl
    // - above-home and home is not set
    // - above-origin and origin is not set
    bool get_alt_cm(AltFrame desired_frame, int32_t &ret_alt_cm) const WARN_IF_UNUSED;
    // same as get_alt_cm but in metres:
    bool get_alt_m(AltFrame desired_frame, float &ret_alt) const WARN_IF_UNUSED;

    // get altitude frame
    AltFrame get_alt_frame() const;

    // converts altitude to new frame
    // returns false on failure to convert which can only happen if the original frame or desired frame is:
    // - above-terrain and the terrain database can't supply terrain height amsl
    // - above-home and home is not set
    // - above-origin and origin is not set
    bool change_alt_frame(AltFrame desired_frame);

    // copy altitude and its frame of other Location object:
    void copy_alt_from(const Location &other);

    // get position as a vector (in cm) from origin (x,y only or
    // x,y,z) return false on failure to get the vector which can only
    // happen if the EKF origin has not been set yet x, y and z are in
    // centimetres.  If this method returns false then vec_ne is
    // unmodified.
    template<typename T>
    bool get_vector_xy_from_origin_NE_cm(T &vec_ne) const WARN_IF_UNUSED;
    // converts location to a vector from origin; if this method returns
    // false then vec_neu is unmodified
    template<typename T>
    bool get_vector_from_origin_NEU_cm(T &vec_neu) const WARN_IF_UNUSED;
    // same as get_vector_from_origin_NEU_cm, but only here so we can
    // continue to use it in LUA scripts:
    template<typename T>
    bool get_vector_from_origin_NEU(T &vec_neu) const WARN_IF_UNUSED;

    // get position as a vector (in metres) from origin (x,y only or
    // x,y,z) return false on failure to get the vector which can only
    // happen if the EKF origin has not been set yet x, y and z are in
    // metres.  If this method returns false then vec_ne is
    // unmodified.
    template<typename T>
    bool get_vector_xy_from_origin_NE_m(T &vec_ne) const;
    template<typename T>
    bool get_vector_from_origin_NEU_m(T &vec_neu) const;

    // return horizontal distance in meters between two locations
    ftype get_distance(const Location &loc2) const;

    // return the altitude difference in meters taking into account
    // alt frame.  if loc2 is below this location then "distance" will
    // be positive.  ie. this method returns how far above loc2 this
    // location is.
    bool get_height_above(const Location &loc2, ftype &distance) const WARN_IF_UNUSED;

    // return the distance in meters in North/East/Down plane as a N/E/D vector to loc2
    // NOT CONSIDERING ALT FRAME!
    Vector3f get_distance_NED(const Location &loc2) const;
    Vector3p get_distance_NED_postype(const Location &loc2) const;
    Vector3d get_distance_NED_double(const Location &loc2) const;

    // return the distance in meters in North/East/Down plane as a N/E/D vector to loc2 considering alt frame, if altitude cannot be resolved down distance is 0
    Vector3f get_distance_NED_alt_frame(const Location &loc2) const;

    // return the distance in meters in North/East plane as a N/E vector to loc2
    Vector2f get_distance_NE(const Location &loc2) const;
    Vector2p get_distance_NE_postype(const Location &loc2) const;
    Vector2d get_distance_NE_double(const Location &loc2) const;
    Vector2F get_distance_NE_ftype(const Location &loc2) const;

    // extrapolate latitude/longitude given distances (in meters) north and east
    static void offset_latlng(int32_t &lat, int32_t &lng, ftype ofs_north, ftype ofs_east);
    void offset(ftype ofs_north, ftype ofs_east);
    // extrapolate latitude/longitude given distances (in meters) north
    // and east. Note that this is metres, *even for the altitude*.
    void offset(const Vector3p &ofs_ned);
    void offset_up_cm(int32_t alt_offset_cm) {
        alt += alt_offset_cm;
    }
    void offset_up_m(float alt_offset_m) {
        alt += alt_offset_m * 100;
    }

    // extrapolate latitude/longitude given bearing and distance
    void offset_bearing(ftype bearing_deg, ftype distance);
    
    // extrapolate latitude/longitude given bearing, pitch and distance
    void offset_bearing_and_pitch(ftype bearing_deg, ftype pitch_deg, ftype distance);

    // longitude_scale - returns the scaler to compensate for
    // shrinking longitude as you move north or south from the equator
    // Note: this does not include the scaling to convert
    // longitude/latitude points to meters or centimeters
    static ftype longitude_scale(int32_t lat);

    bool is_zero(void) const WARN_IF_UNUSED;

    void zero(void);

    // return the bearing in radians, from 0 to 2*Pi
    ftype get_bearing(const Location &loc2) const;

    // return bearing in centi-degrees from location to loc2, return is 0 to 35999
    int32_t get_bearing_to(const Location &loc2) const {
        return int32_t(rad_to_cd(get_bearing(loc2)) + 0.5);
    }

    // check if lat and lng match. Ignore altitude and options
    bool same_latlon_as(const Location &loc2) const;

    // check if altitude matches.
    bool same_alt_as(const Location &loc2) const;

    // check if lat, lng, and alt match.
    bool same_loc_as(const Location &loc2) const {
        return same_latlon_as(loc2) && same_alt_as(loc2);
    }

    /*
     * convert invalid waypoint with useful data. return true if location changed
     */
    bool sanitize(const Location &defaultLoc);

    // return true when lat and lng are within range
    bool check_latlng() const;

    // see if location is past a line perpendicular to
    // the line between point1 and point2 and passing through point2.
    // If point1 is our previous waypoint and point2 is our target waypoint
    // then this function returns true if we have flown past
    // the target waypoint
    bool past_interval_finish_line(const Location &point1, const Location &point2) const;

    /*
      return the proportion we are along the path from point1 to
      point2, along a line parallel to point1<->point2.
      This will be more than 1 if we have passed point2
     */
    float line_path_proportion(const Location &point1, const Location &point2) const;

    // update altitude and alt-frame base on this location's horizontal position between point1 and point2
    // this location's lat,lon is used to calculate the alt of the closest point on the line between point1 and point2
    // origin and destination's altitude frames must be the same
    // this alt-frame will be updated to match the destination alt frame
    void linearly_interpolate_alt(const Location &point1, const Location &point2);

    bool initialised() const { return (lat !=0 || lng != 0 || alt != 0); }
    // return true if alt is *exactly* zero.  This is not intended to
    // be used for anything except magically changing a 0-altitude
    // passed via mavlink into the current vehicle's altitude
    bool alt_is_zero() const { return alt == 0; }

    // wrap longitude at -180e7 to 180e7
    static int32_t wrap_longitude(int64_t lon);

    // limit latitude to -90e7 to 90e7
    static int32_t limit_lattitude(int32_t lat);
    
    // get lon1-lon2, wrapping at -180e7 to 180e7
    static int32_t diff_longitude(int32_t lon1, int32_t lon2);

private:

    // scaling factor from 1e-7 degrees to meters at equator
    // == 1.0e-7 * DEG_TO_RAD * RADIUS_OF_EARTH
    static constexpr float LOCATION_SCALING_FACTOR = LATLON_TO_M;
    // inverse of LOCATION_SCALING_FACTOR
    static constexpr float LOCATION_SCALING_FACTOR_INV = LATLON_TO_M_INV;
};
