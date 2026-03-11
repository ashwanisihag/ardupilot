#include "Copter.h"
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <AP_Terrain/AP_Terrain.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <stdarg.h>

extern Copter copter;

// --- local helper: offset a Location by N/E meters ---
static  void offset_ne(Location& location, float dn, float de)
{
    const float R = 6378137.0f;   // Earth radius in meters (WGS84)

    float lat_rad = radians(location.lat * 1.0e-7f);  // degrees * 1e-7 → deg → rad

    const float dLat = dn / R;
    const float dLon = de / (R * cosf(lat_rad));

    lat_rad += dLat;
    float lon_rad = radians(location.lng * 1.0e-7f) + dLon;

    location.lat = degrees(lat_rad) * 1.0e7f;
    location.lng = degrees(lon_rad) * 1.0e7f;
}


// -----------------------------------------------------------------------------
// Parameter group for "POI_*"
// -----------------------------------------------------------------------------
const AP_Param::GroupInfo ModePoi::var_info[] = {
    // @Param: POI_DIST_MAX
    // @DisplayName: Mount POI distance max
    // @Description: POI's max distance (in meters) from the vehicle
    // @Units: m
    // @Range: 0 10000
    // @User: Standard
    AP_GROUPINFO("DIST_MAX",  1, ModePoi, _params.dist_max, 10000.0f),

    // @Param: POI_STEP_M
    // @DisplayName: POI step size
    // @Description: Step size used along the ray (meters). If 0, use default/fallback
    // @Units: m
    // @Range: 0 100
    // @User: Advanced
    AP_GROUPINFO("STEP_M",    2, ModePoi, _params.step_m,   0.0f),

    // @Param: POI_SET_ROI
    // @DisplayName: Lock ROI on POI
    // @Description: If 1, set mount ROI to the solved POI
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("SET_ROI",   3, ModePoi, _params.set_roi,  1),

    // @Param: POI_AUTO_ONCE
    // @DisplayName: Auto compute once
    // @Description: If 1, compute POI once on entry and return to previous mode
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("AUTO_ONCE", 4, ModePoi, _params.auto_once,0),

    AP_GROUPEND
};

// -----------------------------------------------------------------------------
// Constructor
// -----------------------------------------------------------------------------
ModePoi::ModePoi()
{
    AP_Param::setup_object_defaults(this, var_info);
}

// -----------------------------------------------------------------------------
// Init
// -----------------------------------------------------------------------------
bool ModePoi::init(bool ignore_checks)
{
    (void)ignore_checks;

    if (!ahrs.healthy()) {
        say("POI: AHRS not healthy");
        return false;
    }

    Location curr;
    if (!ahrs.get_location(curr)) {
        say("POI: vehicle location unavailable");
        return false;
    }

    _did_once = false;
    //_return_mode = static_cast<Mode::Number>(copter.get_mode());

    say("POI mode started");
    return true;
}

// -----------------------------------------------------------------------------
// Run loop
// -----------------------------------------------------------------------------
void ModePoi::run()
{
    hold_attitude_only();

    if (_params.auto_once && !_did_once) {
        Location poi;
        if (compute_poi(poi)) {
            say("POI %.7f, %.7f, %.2f ASL",
                poi.lat * 1.0e-7f, poi.lng * 1.0e-7f, poi.alt * 0.01f);
            camera_feedback(poi);

            if (_params.set_roi) {
                if (AP_Mount *mnt = AP::mount()) {
                    mnt->set_roi_target(0, poi);
                }
            }

            _did_once = true;
            set_mode(_return_mode, ModeReason::SCRIPTING);
        }
    }
}

// -----------------------------------------------------------------------------
// Helpers
// -----------------------------------------------------------------------------
void ModePoi::hold_attitude_only()
{
    // Simple attitude hold — zero body-frame rates
    attitude_control->input_rate_bf_roll_pitch_yaw_rads(0.0f, 0.0f, 0.0f);
}

float ModePoi::step_size_m() const
{
    const float p = _params.step_m;
    if (p > 0.0f) {
        return MAX(0.5f, p);
    }
    return 30.0f; // fallback if user leaves STEP_M = 0
}

bool ModePoi::get_mount_attitude_deg(float &pitch_deg, float &yaw_ef_deg) const
{
    AP_Mount *mnt = AP::mount();
    if (!mnt) {
        return false;
    }

    float roll_bf_deg = 0.0f;
    float pitch_bf_deg = 0.0f;
    float yaw_bf_deg = 0.0f;

    if (!mnt->get_attitude_euler(0, roll_bf_deg, pitch_bf_deg, yaw_bf_deg)) {
        return false;
    }

    const float yaw_veh_deg = degrees(ahrs.get_yaw());   // radians -> deg
    yaw_ef_deg = wrap_180(yaw_bf_deg + yaw_veh_deg);     // mount yaw + vehicle yaw
    pitch_deg  = pitch_bf_deg;
    return true;
}

bool ModePoi::terrain_height_amsl(const Location &loc, float &h_amsl) const
{
    AP_Terrain *ter = AP::terrain();
    if (!ter) {
        return false;
    }
    Location loc_copy = loc;

    if (ter->height_amsl(loc_copy, h_amsl, true)) {
        return true;
    }
    return ter->height_amsl(loc_copy, h_amsl, false);
}

bool ModePoi::compute_poi(Location &poi_out)
{
    Location vehicle_loc;
    if (!ahrs.get_location(vehicle_loc)) {
        say("POI: vehicle pos unavailable");
        return false;
    }

    Location test_loc = vehicle_loc;
    Location prev_loc = test_loc;

    float terrain_amsl = 0.0f, prev_terrain_amsl = 0.0f;
    if (!terrain_height_amsl(test_loc, terrain_amsl)) {
        say("POI: terrain unavailable");
        return false;
    }
    prev_terrain_amsl = terrain_amsl;

    float pitch_deg = 0.0f, yaw_ef_deg = 0.0f;
    if (!get_mount_attitude_deg(pitch_deg, yaw_ef_deg)) {
        say("POI: gimbal attitude unavailable");
        return false;
    }

    const float dist_max_m = MAX(0.0f, float(_params.dist_max));
    const float step_m     = step_size_m();
    float total            = 0.0f;

    while ((total < dist_max_m) && (test_loc.alt * 0.01f > terrain_amsl)) {
        total += step_m;

        prev_loc = test_loc;
        prev_terrain_amsl = terrain_amsl;

        const float pitch_rad = radians(pitch_deg);
        const float horiz_m   = step_m * cosf(pitch_rad);
        const float dz_m      = -step_m * sinf(pitch_rad); // down positive

        const float yaw_rad = radians(yaw_ef_deg);
        const float dn = cosf(yaw_rad) * horiz_m;
        const float de = sinf(yaw_rad) * horiz_m;
        offset_ne(test_loc, dn, de);

        test_loc.alt += (int32_t)lrintf(dz_m * 100.0f);

        if (!terrain_height_amsl(test_loc, terrain_amsl)) {
            say("POI: terrain fetch fail");
            return false;
        }
    }

    if (total >= dist_max_m) {
        say("POI: no terrain within %.0f m", (double)dist_max_m);
        return false;
    }

    const float f_prev = prev_loc.alt * 0.01f - prev_terrain_amsl;
    const float f_curr = test_loc.alt * 0.01f - terrain_amsl;

    float t = 0.0f;
    const float denom = (f_prev - f_curr);
    if (fabsf(denom) > 1e-6f) {
        t = constrain_float(f_prev / denom, 0.0f, 1.0f);
    }

    poi_out = prev_loc;

    const float pitch_rad = radians(pitch_deg);
    const float seg_len_m = step_m * t;
    const float horiz_m   = seg_len_m * cosf(pitch_rad);
    const float dz_m      = -(seg_len_m) * sinf(pitch_rad);

    {
        const float yaw_rad = radians(yaw_ef_deg);
        const float dn = cosf(yaw_rad) * horiz_m;
        const float de = sinf(yaw_rad) * horiz_m;
        offset_ne(poi_out, dn, de);
    }
    poi_out.alt += (int32_t)lrintf(dz_m * 100.0f);

    return true;
}

bool ModePoi::camera_feedback(const Location &poi)
{
    const uint64_t now_us = AP_HAL::micros64();
    const uint16_t idx = ++_capture_idx;

    for (uint8_t i = 0; i < MAVLINK_COMM_NUM_BUFFERS; i++) {
        mavlink_msg_camera_feedback_send(
            (mavlink_channel_t)i,
            now_us,            // time_usec
            poi.lat,           // lat (degE7)
            poi.lng,           // lon (degE7)
            poi.alt * 0.01f,   // alt_msl (m)
            0.0f,              // alt_rel (m)
            0.0f, 0.0f, 0.0f,  // roll, pitch, yaw (deg)
            0.0f,              // foc_len (mm)
            idx,               // img_idx
            0,                 // target_system
            0,                 // cam_idx
            0,                 // flags
            idx                // completed_captures
        );
    }
    return true;
}

void ModePoi::say(const char *fmt, ...)
{
    char buf[96];
    va_list ap;
    va_start(ap, fmt);
    hal.util->vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    gcs().send_text(MAV_SEVERITY_INFO, "%s", buf);
}
