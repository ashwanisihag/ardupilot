// ArduCopter/mode_snake.cpp
#include "Copter.h"
#include <AP_Math/AP_Math.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Param/AP_Param.h>
#include <math.h>

// ----------------------------
// SNAKE mode: forward speed hold with alternating roll, altitude hold
// ----------------------------

// ***** TUNABLES *****

// Extra forward push on RETURN and shorter brake zone
#define SNAKE_RETURN_EXTRA_PITCH_DEG   3.0f   // adds nose-down on return (deg); try 2–5
#define SNAKE_RETURN_BRAKE_DIST_M      30.0f  // delay braking on return (m); try 20–40

// Hard-code extra yaw beyond 180° at turnaround (positive => more than 180°)
#define SNAKE_TURN_EXTRA_DEG           0.0f   // e.g. 30 => ~210° total turn

// Target speed during calibration window (actual EXEC uses SNAKE_SPEED param)
#define SNAKE_TARGET_SPEED_MS          5.0f
#define SNAKE_TARGET_SPEED_CMS         (SNAKE_TARGET_SPEED_MS * 100.0f)

// "Calibration" window: run EXEC-like control to estimate pitch for target speed
#define SNAKE_CALIB_EXEC_WINDOW_MS     10000U

// Pitch/roll limits & dynamics
#define SNAKE_MAX_PITCH_DEG            25.0f     // |pitch| cap (nose-down negative)
#define SNAKE_MAX_PITCH_RATE_DPS       10.0f     // forward-tilt rate limit (deg/s)
#define SNAKE_OSC_PERIOD_MS            3000U     // switch roll direction every 3s

// Estimation & PI gains (LPF for speed during window)
#define SNAKE_GS_LPF_ALPHA             0.15f     // forward-speed LPF
#define SNAKE_INT_CLAMP                1.0f      // anti-windup clamp (m/s*s)

// EXECUTE-phase speed-hold PI (drives pitch)
#define SNAKE_EXEC_KP                  0.8f
#define SNAKE_EXEC_KI                  0.2f

// Post-lock pitch tweak
#define SNAKE_EXEC_PITCH_BIAS_DEG      1.0f      // small pitch-up bias after lock

// Reporting
#define SNAKE_REPORT_STEP_M            100.0f    // announce every 100 m

// Integration/finish helpers
#define SNAKE_BRAKE_DIST_M             100.0f    // brake taper distance (outbound leg)

// Return-to-start option
#define SNAKE_ENABLE_RETURN            1
#define SNAKE_YAW_KP                   2.0f
#define SNAKE_MAX_YAW_RATE_DPS         60.0f
#define SNAKE_YAW_LOCK_TOL_DEG         5.0f

static float constrain_deg(float deg, float minv, float maxv)
{
    if (deg < minv) return minv;
    if (deg > maxv) return maxv;
    return deg;
}

// ---------------- Param registration ----------------
// NOTE: names are UNPREFIXED here; subgroup adds "SNAKE_" prefix in Parameters.cpp
const AP_Param::GroupInfo ModeSnake::var_info[] = {
    // @Param: DIST
    // @DisplayName: Snake outbound distance
    // @Units: m
    AP_GROUPINFO("DIST",  1, ModeSnake, _p_dist_m,   190.0f),

    // @Param: SPEED
    // @DisplayName: Snake target forward speed
    // @Units: m/s
    AP_GROUPINFO("SPEED", 2, ModeSnake, _p_speed_ms, 5.0f),

    // @Param: ROLL
    // @DisplayName: Snake roll amplitude
    // @Units: deg
    AP_GROUPINFO("ROLL",  3, ModeSnake, _p_roll_deg, 12.0f),

    AP_GROUPEND
};

ModeSnake::ModeSnake()
{
    AP_Param::setup_object_defaults(this, var_info);
}

// Forward speed (body-X) in cm/s; positive = along nose direction
float ModeSnake::get_forward_speed_cms() const
{
    const float yaw = AP::ahrs().get_yaw(); // radians

    Vector3f vel_ned;
    if (AP::ahrs().get_velocity_NED(vel_ned)) {
        const float vn = vel_ned.x;
        const float ve = vel_ned.y;
        const float v_forward_ms = vn * cosf(yaw) + ve * sinf(yaw);
        return v_forward_ms * 100.0f;
    }

    if (AP::gps().status() >= AP_GPS::GPS_OK_FIX_3D) {
        const float gs_cms = AP::gps().ground_speed();
        const float course_rad = radians(AP::gps().ground_course() * 0.01f);
        const float sign = cosf(course_rad - yaw);
        return gs_cms * sign;
    }
    return 0.0f;
}

bool ModeSnake::init(bool ignore_checks)
{
    if (!pos_control->is_active_U()) {
        pos_control->init_U_controller();
    }
    pos_control->set_max_speed_accel_U_cm(-(get_pilot_speed_dn_ms()*100.0f), g.pilot_speed_up_cms, g.pilot_accel_u_cmss);
    pos_control->set_correction_speed_accel_U_cm   (-(get_pilot_speed_dn_ms()*100.0f), g.pilot_speed_up_cms, g.pilot_accel_u_cmss);

    _phase = SnakePhase::CALIBRATE;

    _calib_pitch_deg_cmd = 0.0f;
    _calib_pi_int = 0.0f;
    _gs_lpf_cms = 0.0f;

    _phase_start_ms = AP_HAL::millis();
    _gs_over_start_ms = 0U;
    _last_toggle_ms = _phase_start_ms;
    _yaw_dir = +1;

    _calib_dist_m = 0.0f;
    _remaining_dist_m = 0.0f;
    _v_exec_ms = 0.0f;
    _dist_m = 0.0f;
    _next_report_m = SNAKE_REPORT_STEP_M;
    _exec_time_ms = 0U;

    _start_yaw_rad = AP::ahrs().get_yaw();
    _return_yaw_rad = 0.0f;
    _outbound_total_m = 0.0f;
    _return_dist_m = 0.0f;

    attitude_control->reset_rate_controller_I_terms();

    gcs().send_text(MAV_SEVERITY_INFO,
        "Snake: init CALIB-WINDOW 10s (speed-hold to %.1f m/s, AltHold locked)",
        (double)SNAKE_TARGET_SPEED_MS);
    return true;
}

void ModeSnake::run()
{
    // Read parameters (bounded)
    const float target_speed_ms = constrain_float(_p_speed_ms.get(), 0.5f, 20.0f);
    const float roll_amp_deg    = constrain_float(_p_roll_deg.get(), 0.0f, 35.0f);

    pos_control->set_max_speed_accel_U_cm(
        -(get_pilot_speed_dn_ms()*100.0f),   // m/s -> cm/s
        g.pilot_speed_up_cms,                // already cm/s
        g.pilot_accel_u_cmss                 // cm/s^2
    );

    float target_roll_rad = 0.0f;
    float target_pitch_rad = 0.0f;
    float target_yaw_rate_rads = 0.0f;

    const uint32_t now = AP_HAL::millis();
    const float dt = G_Dt;

    static bool z_latched = false;
    if (!z_latched) {
        pos_control->set_pos_target_U_from_climb_rate_m(0.0f);
        z_latched = true;
    }

    // Interpret SNAKE_DIST as outbound path length (meters)
    const float travel_dist_m = fmaxf(0.0f, _p_dist_m.get());

    switch (_phase) {

    // -------- CALIBRATE: run PI on speed for a window to find pitch --------
    case SnakePhase::CALIBRATE: {
        if (now - _last_toggle_ms >= SNAKE_OSC_PERIOD_MS) {
            _yaw_dir = -_yaw_dir;
            _last_toggle_ms = now;
            gcs().send_text(MAV_SEVERITY_DEBUG, "Snake: toggle roll_dir=%d", _yaw_dir);
        }
        const float roll_deg = ((float)_yaw_dir) * roll_amp_deg;
        target_roll_rad = radians(roll_deg);

        const float v_cms_inst = get_forward_speed_cms();
        _gs_lpf_cms = (1.0f - SNAKE_GS_LPF_ALPHA) * _gs_lpf_cms + SNAKE_GS_LPF_ALPHA * v_cms_inst;
        const float v_fwd_ms = fmaxf(0.0f, _gs_lpf_cms * 0.01f);

        const float v_err = (target_speed_ms - v_fwd_ms);
        _calib_pi_int += v_err * dt;
        _calib_pi_int = constrain_float(_calib_pi_int, -SNAKE_INT_CLAMP, SNAKE_INT_CLAMP);

        float pitch_rate_deg_s = -(SNAKE_EXEC_KP * v_err + SNAKE_EXEC_KI * _calib_pi_int);
        pitch_rate_deg_s = constrain_float(pitch_rate_deg_s, -SNAKE_MAX_PITCH_RATE_DPS, SNAKE_MAX_PITCH_RATE_DPS);

        _calib_pitch_deg_cmd += pitch_rate_deg_s * dt;
        _calib_pitch_deg_cmd = constrain_deg(_calib_pitch_deg_cmd, -SNAKE_MAX_PITCH_DEG, 0.0f);

        float base_pitch_deg = _calib_pitch_deg_cmd + SNAKE_EXEC_PITCH_BIAS_DEG;
        base_pitch_deg *= cosf(fabsf(target_roll_rad));
        target_pitch_rad = radians(base_pitch_deg);
        target_yaw_rate_rads = 0.0f;

        _calib_dist_m += v_fwd_ms * dt;

        const bool window_done = (now - _phase_start_ms) >= SNAKE_CALIB_EXEC_WINDOW_MS;
        if (window_done) {
            // lock: use param speed for EXECUTE
            _v_exec_ms = target_speed_ms;
            _calib_pitch_deg_cmd = constrain_deg(_calib_pitch_deg_cmd, -SNAKE_MAX_PITCH_DEG, 0.0f);

            _remaining_dist_m = fmaxf(0.0f, travel_dist_m - _calib_dist_m);
            _dist_m = 0.0f;

            const float speed_for_time = (_v_exec_ms > 0.3f) ? _v_exec_ms : target_speed_ms;
            _exec_time_ms = (uint32_t)((_remaining_dist_m / fmaxf(speed_for_time, 0.3f)) * 1000.0f);

            const float total_so_far = _calib_dist_m;
            const float step = SNAKE_REPORT_STEP_M;
            _next_report_m = ceilf(total_so_far / step) * step;
            if (_next_report_m < total_so_far + 1e-3f) { _next_report_m += step; }

            gcs().send_text(MAV_SEVERITY_INFO,
                "Snake: window end: pitch=%.1fdeg v=%.2f m/s, rem=%.0f m -> %.1f s",
                _calib_pitch_deg_cmd, _v_exec_ms, _remaining_dist_m, _exec_time_ms * 0.001f);

            if (_remaining_dist_m <= 0.0f) {
                _outbound_total_m = _calib_dist_m;
#if SNAKE_ENABLE_RETURN
                _phase = SnakePhase::TURNAROUND;
                _phase_start_ms = now;
                _return_yaw_rad = wrap_PI(AP::ahrs().get_yaw()
                                          + radians(180.0f + SNAKE_TURN_EXTRA_DEG));
                gcs().send_text(MAV_SEVERITY_INFO,
                    "Snake: outbound done in window (%.0f m). Turning...", _outbound_total_m);
#else
                _phase = SnakePhase::DONE;
                gcs().send_text(MAV_SEVERITY_INFO,
                    "Snake: target reached within 10s window (%.0f m)", total_so_far);
#endif
                target_pitch_rad = 0.0f;
                target_roll_rad = 0.0f;
                target_yaw_rate_rads = 0.0f;
                break;
            }

            _phase = SnakePhase::EXECUTE;
            _phase_start_ms = now;
            _last_toggle_ms = now;
            _yaw_dir = +1;
        }
        break;
    }

    // -------- EXECUTE: hold speed using fixed pitch, integrate distance --------
    case SnakePhase::EXECUTE: {
        if (now - _last_toggle_ms >= SNAKE_OSC_PERIOD_MS) {
            _yaw_dir = -_yaw_dir;
            _last_toggle_ms = now;
            gcs().send_text(MAV_SEVERITY_DEBUG, "Snake: toggle roll_dir=%d", _yaw_dir);
        }
        const float roll_deg = ((float)_yaw_dir) * roll_amp_deg;
        target_roll_rad = radians(roll_deg);

        float base_pitch_exec_deg = _calib_pitch_deg_cmd + SNAKE_EXEC_PITCH_BIAS_DEG;
        base_pitch_exec_deg *= cosf(fabsf(target_roll_rad));

        const float v_ms_for_integrator = fmaxf(0.0f, get_forward_speed_cms() * 0.01f);
        _dist_m += v_ms_for_integrator * dt;

        const float total_m = _calib_dist_m + _dist_m;

        const float rem_total = fmaxf(0.0f, travel_dist_m - total_m);
        const float brake_scale = (SNAKE_BRAKE_DIST_M > 1e-3f)
            ? fminf(1.0f, rem_total / SNAKE_BRAKE_DIST_M) : 1.0f;
        const float pitch_exec_deg = base_pitch_exec_deg * brake_scale;

        target_pitch_rad = radians(pitch_exec_deg);
        target_yaw_rate_rads = 0.0f;

        while (total_m + 1e-3f >= _next_report_m) {
            gcs().send_text(MAV_SEVERITY_INFO, "Snake: distance %.0f m", _next_report_m);
            _next_report_m += SNAKE_REPORT_STEP_M;
        }

        // Distance-only cutoff (no time-based early stop)
        if (total_m >= travel_dist_m) {
            _outbound_total_m = total_m;
#if SNAKE_ENABLE_RETURN
            _phase = SnakePhase::TURNAROUND;
            _phase_start_ms = now;
            _return_yaw_rad = wrap_PI(AP::ahrs().get_yaw()
                                      + radians(180.0f + SNAKE_TURN_EXTRA_DEG));
            _return_dist_m = 0.0f;
            gcs().send_text(MAV_SEVERITY_INFO,
                "Snake: outbound finished (%.0f m). Turning...", _outbound_total_m);
#else
            _phase = SnakePhase::DONE;
            gcs().send_text(MAV_SEVERITY_INFO,
                "Snake: EXECUTE finished (%.0f m total)", total_m);
#endif
            target_pitch_rad = 0.0f;
            target_roll_rad = 0.0f;
            target_yaw_rate_rads = 0.0f;
        }
        break;
    }

    // -------- TURNAROUND: yaw towards return heading --------
    case SnakePhase::TURNAROUND: {
        target_roll_rad = 0.0f;
        target_pitch_rad = 0.0f;

        const float yaw_now = AP::ahrs().get_yaw();
        float yaw_err = wrap_PI(_return_yaw_rad - yaw_now);

        float yaw_rate_rads = SNAKE_YAW_KP * yaw_err;
        yaw_rate_rads = constrain_float(
            yaw_rate_rads,
            radians(-SNAKE_MAX_YAW_RATE_DPS),
            radians(+SNAKE_MAX_YAW_RATE_DPS)
        );
        target_yaw_rate_rads = yaw_rate_rads;

        if (fabsf(degrees(yaw_err)) <= SNAKE_YAW_LOCK_TOL_DEG) {
            _phase = SnakePhase::RETURN;
            _phase_start_ms = now;
            _last_toggle_ms = now;
            _yaw_dir = +1;
            _return_dist_m = 0.0f;
            _next_report_m = SNAKE_REPORT_STEP_M;
            gcs().send_text(MAV_SEVERITY_INFO, "Snake: turnaround complete, returning %.0f m", _outbound_total_m);
        }
        break;
    }

    // -------- RETURN: same as EXECUTE but with extra push & later braking --------
    case SnakePhase::RETURN: {
        if (now - _last_toggle_ms >= SNAKE_OSC_PERIOD_MS) {
            _yaw_dir = -_yaw_dir;
            _last_toggle_ms = now;
            gcs().send_text(MAV_SEVERITY_DEBUG, "Snake: toggle roll_dir=%d", _yaw_dir);
        }
        const float roll_deg = ((float)_yaw_dir) * roll_amp_deg;
        target_roll_rad = radians(roll_deg);

        float base_pitch_exec_deg = _calib_pitch_deg_cmd + SNAKE_EXEC_PITCH_BIAS_DEG;
        base_pitch_exec_deg *= cosf(fabsf(target_roll_rad));

        // Extra forward push on the RETURN leg
        base_pitch_exec_deg -= SNAKE_RETURN_EXTRA_PITCH_DEG;

        const float v_ms_for_integrator = fmaxf(0.0f, get_forward_speed_cms() * 0.01f);
        _return_dist_m += v_ms_for_integrator * dt;

        const float rem_return = fmaxf(0.0f, _outbound_total_m - _return_dist_m);
        const float brake_scale = (SNAKE_RETURN_BRAKE_DIST_M > 1e-3f)
            ? fminf(1.0f, rem_return / SNAKE_RETURN_BRAKE_DIST_M) : 1.0f;
        const float pitch_exec_deg = base_pitch_exec_deg * brake_scale;

        target_pitch_rad = radians(pitch_exec_deg);
        target_yaw_rate_rads = 0.0f;

        while (_return_dist_m + 1e-3f >= _next_report_m) {
            gcs().send_text(MAV_SEVERITY_INFO, "Snake: returning distance %.0f m", _next_report_m);
            _next_report_m += SNAKE_REPORT_STEP_M;
        }

        if (_return_dist_m >= _outbound_total_m) {
            _phase = SnakePhase::DONE;
            gcs().send_text(MAV_SEVERITY_INFO, "Snake: return finished (≈%.0f m back)", _outbound_total_m);
            target_pitch_rad = 0.0f;
            target_roll_rad = 0.0f;
            target_yaw_rate_rads = 0.0f;
        }
        break;
    }

    case SnakePhase::DONE: {
        target_pitch_rad = 0.0f;
        target_roll_rad = 0.0f;
        target_yaw_rate_rads = 0.0f;
        break;
    }
    } // switch

    const float roll_cd = degrees(target_roll_rad) * 100.0f;
    const float pitch_cd = degrees(target_pitch_rad) * 100.0f;
    const float yaw_rate_cds = degrees(target_yaw_rate_rads) * 100.0f;

    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_cd(
        roll_cd, pitch_cd, yaw_rate_cds
    );

    pos_control->set_pos_target_U_from_climb_rate_m(0.0f);
    pos_control->update_U_controller();
}
