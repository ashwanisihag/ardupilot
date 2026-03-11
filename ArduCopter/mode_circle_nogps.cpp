// ArduCopter/mode_circle_nogps.cpp

#include "Copter.h"
#include <AP_Math/AP_Math.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Param/AP_Param.h>
#include <math.h>

// ---------------- helper ----------------
static float constrain_deg(float deg, float minv, float maxv)
{
    if (deg < minv) return minv;
    if (deg > maxv) return maxv;
    return deg;
}

// ---------------- Param registration ----------------
// (All fields are members of ModeCircleNoGPS declared in mode.h)
// mode_circle_nogps.cpp  (replace just the table)
const AP_Param::GroupInfo ModeCircleNoGPS::var_info[] = {
    // Name, idx, class, field, default
    AP_GROUPINFO("SPEED",      1, ModeCircleNoGPS, target_speed_ms,       5.0f),
    AP_GROUPINFO("MAX_P",      2, ModeCircleNoGPS, max_pitch_deg,        25.0f),
    AP_GROUPINFO("MAX_PR",     3, ModeCircleNoGPS, max_pitchrate_dps,    10.0f),

    AP_GROUPINFO("GS_LPF",     4, ModeCircleNoGPS, gs_lpf_alpha,          0.15f),
    AP_GROUPINFO("KP",         5, ModeCircleNoGPS, exec_kp,               0.8f),
    AP_GROUPINFO("KI",         6, ModeCircleNoGPS, exec_ki,               0.2f),
    AP_GROUPINFO("INTLIM",     7, ModeCircleNoGPS, int_clamp,             1.0f),
    AP_GROUPINFO("BIAS",       8, ModeCircleNoGPS, exec_pitch_bias_deg,   1.0f),

    AP_GROUPINFO("RADIUS",     9, ModeCircleNoGPS, radius_m,             60.0f),
    AP_GROUPINFO("DIR",       10, ModeCircleNoGPS, ccw,                   1),
    AP_GROUPINFO("MAX_YR",    11, ModeCircleNoGPS, max_yawrate_dps,      60.0f),
    AP_GROUPINFO("MIN_OMEGA", 12, ModeCircleNoGPS, min_speed_for_omega,   0.3f),

    AP_GROUPINFO("REPORT_S",  13, ModeCircleNoGPS, report_every_sec,     10),
    AP_GROUPEND
};


// ---------------- ctor ----------------
ModeCircleNoGPS::ModeCircleNoGPS()
{
    // Parameters will appear in GCS as: CNGPS_SPEED, CNGPS_RADIUS, ...
    AP_Param::setup_object_defaults(this, var_info);
}

// ---------------- utilities ----------------

// Forward speed (body-X) in cm/s; positive = along nose direction
float ModeCircleNoGPS::get_forward_speed_cms() const
{
    const float yaw = AP::ahrs().get_yaw(); // radians

    // Prefer EKF NED velocity if available
    Vector3f vel_ned;
    if (AP::ahrs().get_velocity_NED(vel_ned)) {
        const float vn = vel_ned.x;
        const float ve = vel_ned.y;
        const float v_forward_ms = vn * cosf(yaw) + ve * sinf(yaw); // body-X projection
        return v_forward_ms * 100.0f;
    }

    // Fallback to GPS groundspeed/course if valid
    if (AP::gps().status() >= AP_GPS::GPS_OK_FIX_3D) {
        const float gs_cms    = AP::gps().ground_speed();
        const float course_rad= radians(AP::gps().ground_course() * 0.01f);
        const float sign      = cosf(course_rad - yaw);             // projection onto body-X
        return gs_cms * sign;
    }

    // Otherwise unknown -> 0
    return 0.0f;
}

// ---------------- mode API ----------------
bool ModeCircleNoGPS::init(bool ignore_checks)
{
    if (!pos_control->is_active_U()) {
        pos_control->init_U_controller();
    }
    pos_control->set_max_speed_accel_U_cm(
        -(get_pilot_speed_dn_ms()*100.0f),  // m/s -> cm/s
        g.pilot_speed_up_cms,               // cm/s
        g.pilot_accel_u_cmss                // cm/s^2
    );
    // simple continuous circle "phase"
    _phase_start_ms = AP_HAL::millis();
    _last_toggle_ms = _phase_start_ms; // reserved for future use

    // Seed LPF with current forward speed to avoid jump on entry
    _gs_lpf_cms = get_forward_speed_cms();

    _calib_pi_int = 0.0f;
    _calib_pitch_deg_cmd = 0.0f;

    attitude_control->reset_rate_controller_I_terms();

    gcs().send_text(MAV_SEVERITY_INFO,
        "Circle NoGPS: R=%.0f m @ %.1f m/s (AltHold locked)",
        (double)radius_m.get(), (double)target_speed_ms.get());
    return true;
}

void ModeCircleNoGPS::run()
{
       pos_control->set_max_speed_accel_U_cm(
        -(get_pilot_speed_dn_ms()*100.0f),  // m/s -> cm/s
        g.pilot_speed_up_cms,               // cm/s
        g.pilot_accel_u_cmss                // cm/s^2
    );
    float target_roll_rad = 0.0f;
    float target_pitch_rad = 0.0f;
    float target_yaw_rate_rads = 0.0f;

    const uint32_t now = AP_HAL::millis();
    const float dt = G_Dt;

    // Latch altitude target once (AltHold-style)
    static bool z_latched = false;
    if (!z_latched) {
        pos_control->set_pos_target_U_from_climb_rate_m(0.0f);
        z_latched = true;
    }

    // -------- Read tunables (with guards) --------
    const float dir_sign            = (ccw.get() >= 1) ? 1.0f : -1.0f;
    const float R                   = MAX(5.0f,  radius_m.get());
    const float v_target            = MAX(0.0f,  target_speed_ms.get());
    const float yawrate_cap_rad     = fabsf(max_yawrate_dps.get()) * radians(1.0f);
    const float pitch_cap_rad       = radians(MAX(1.0f,  max_pitch_deg.get()));
    const float pitch_rate_cap_rad  = radians(MAX(0.1f,  max_pitchrate_dps.get()));
    const float v_min_for_omega     = MAX(0.0f,  min_speed_for_omega.get());
    const float pitch_bias_deg      = exec_pitch_bias_deg.get();
    const float alpha               = constrain_float(gs_lpf_alpha.get(), 0.0f, 1.0f);
    const float Kp                  = exec_kp.get();
    const float Ki                  = exec_ki.get();
    const float I_max               = fabsf(int_clamp.get());

    // -------- Speed estimation (LPF) --------
    const float v_cms_inst = get_forward_speed_cms();
    _gs_lpf_cms = _gs_lpf_cms + alpha * (v_cms_inst - _gs_lpf_cms);  // <-- LPF here
    const float v_ms = fmaxf(0.0f, _gs_lpf_cms * 0.01f);

    // -------- Speed-hold PI -> pitch-rate (deg/s) --------
    const float v_err = (v_target - v_ms);
    _calib_pi_int += Ki * v_err * dt;
    _calib_pi_int = constrain_float(_calib_pi_int, -I_max, I_max);

    float pitch_rate_deg_s = -(Kp * v_err + _calib_pi_int);
    pitch_rate_deg_s = constrain_float(pitch_rate_deg_s,
                                       -degrees(pitch_rate_cap_rad),
                                       +degrees(pitch_rate_cap_rad));

    _calib_pitch_deg_cmd += pitch_rate_deg_s * dt;
    _calib_pitch_deg_cmd = constrain_deg(_calib_pitch_deg_cmd,
                                         -degrees(pitch_cap_rad), 0.0f);

    float pitch_deg = _calib_pitch_deg_cmd + pitch_bias_deg;

    // -------- Circle geometry --------
    const float g_mps2 = 9.80665f;
    const float bank_rad = atanf((v_ms * v_ms) / (R * g_mps2));  // φ = atan(v^2/(R*g))

    // roll is the bank angle, reduced slightly if pitching to maintain total tilt budget
    const float roll_rad = dir_sign * bank_rad * cosf(radians(fabsf(pitch_deg)));
    target_roll_rad = roll_rad;

    // pitch softened by roll coupling
    pitch_deg *= cosf(fabsf(target_roll_rad));
    target_pitch_rad = radians(pitch_deg);

    // yaw rate to trace the circle; gate at low speeds and clamp
    const float v_for_omega = fmaxf(v_ms, v_min_for_omega);
    float desired_omega_rads = v_for_omega / fmaxf(R, 1.0f);
    desired_omega_rads = constrain_float(desired_omega_rads, -yawrate_cap_rad, +yawrate_cap_rad);
    target_yaw_rate_rads = dir_sign * desired_omega_rads;

    // -------- Send attitude + altitude hold --------
    const float roll_cd = degrees(target_roll_rad) * 100.0f;
    const float pitch_cd = degrees(target_pitch_rad) * 100.0f;
    const float yaw_rate_cds = degrees(target_yaw_rate_rads) * 100.0f;

    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_cd(
        roll_cd, pitch_cd, yaw_rate_cds
    );

    pos_control->set_pos_target_U_from_climb_rate_m(0.0f);
    pos_control->update_U_controller();

    // -------- periodic info --------
    static uint32_t last_info = 0;
    const uint32_t report_ms = (uint32_t)MAX(1, report_every_sec.get()) * 1000U;
    if (now - last_info >= report_ms) {
        last_info = now;
        gcs().send_text(MAV_SEVERITY_INFO,
            "Circle NoGPS: v=%.2f m/s, bank=%.1f deg, yawrate=%.1f dps",
            (double)v_ms,
            (double)degrees(fabsf(roll_rad)),
            (double)degrees(fabsf(target_yaw_rate_rads)));
    }
}

