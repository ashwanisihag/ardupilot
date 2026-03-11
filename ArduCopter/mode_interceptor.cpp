// ArduCopter/mode_intercept.cpp

#include "Copter.h"

// --- Intercept GCS lock/lost state (file-scope so init() can reset) ---
static bool intercept_prev_has_target = false;
static uint32_t intercept_last_valid_target_ms = 0;

// ---------------- Param registration for INTERCEPT ----------------

const AP_Param::GroupInfo ModeIntercept::var_info[] = {
    // @Param: ANG_MAX
    // @DisplayName: Intercept maximum lean angle
    // @Description: Limits roll/pitch used to track the target
    // @Units: deg
    // @Range: 5 35
    AP_GROUPINFO("ANG_MAX", 1, ModeIntercept, max_angle_deg, 25.0f),

    // @Param: STOP_M
    // @DisplayName: Intercept stop distance
    // @Description: (unused in current implementation; kept for compatibility)
    // @Units: m
    AP_GROUPINFO("STOP_M", 2, ModeIntercept, stop_dist_m, 2.0f),

    // @Param: K_LAT
    // @DisplayName: Intercept lateral P gain
    // @Description: Roll command gain from normalized x offset
    // @Units: -
    AP_GROUPINFO("K_LAT", 3, ModeIntercept, k_lat, 0.8f),

    // @Param: K_FWD
    // @DisplayName: Intercept forward P gain
    // @Description: Pitch P gain from normalized vertical offset
    // @Units: -
    AP_GROUPINFO("K_FWD", 4, ModeIntercept, k_fwd, 1.0f),

    // @Param: SIGNX
    // @DisplayName: Intercept lateral sign
    // @Description: 1 or -1 to flip lateral response if camera wiring/orientation inverted
    // @Units: -
    // @Values: -1:-1,1:1
    AP_GROUPINFO("SIGNX", 5, ModeIntercept, sign_x, 1),

    // @Param: KI_FWD
    // @DisplayName: Intercept forward I gain
    // @Description: Integral gain on vertical offset (y input)
    // @Units: -
    AP_GROUPINFO("KI_FWD", 6, ModeIntercept, ki_fwd, 0.0f),

    // @Param: KD_FWD
    // @DisplayName: Intercept forward D gain
    // @Description: Derivative gain on vertical offset (y input)
    // @Units: -
    AP_GROUPINFO("KD_FWD", 7, ModeIntercept, kd_fwd, 0.0f),

    // @Param: THR_MAN
    // @DisplayName: Intercept manual throttle when target present
    // @Description: Fixed throttle (0..1) used while INTERCEPT has a valid target. 0 = disabled (use RC throttle).
    // @Units: -
    // @Range: 0 1
    AP_GROUPINFO("THR_MAN", 8, ModeIntercept, fixed_throttle, 0.0f),

    // @Param: LOST_MS
    // @DisplayName: Intercept target lost timeout before LOITER
    // @Description: Time in milliseconds after the last valid target before INTERCEPT switches to LOITER. 0 = no auto-switch.
    // @Units: ms
    // @Range: 0 10000
    AP_GROUPINFO("LOST_MS", 9, ModeIntercept, lost_to_loiter_ms, 3000),

    // @Param: KD_LAT
    // @DisplayName: Intercept shared D damping gain
    // @Description: Shared derivative damping applied to both roll (x) and pitch (y) control. Helps reduce snaking/zig-zag.
    // @Units: -
    // @Range: 0 0.2
    AP_GROUPINFO("KD_LAT", 10, ModeIntercept, kd_lat, 0.0f),

    // @Param: PN_GAIN
    // @DisplayName: Intercept LOS-rate guidance gain
    // @Description: Gain applied to line-of-sight rate (d(x)/dt and d(y)/dt). Higher makes intercept lead more like a missile.
    // @Units: -
    // @Range: 0 2
    AP_GROUPINFO("PN_GAIN", 11, ModeIntercept, pn_gain, 0.0f),

    // @Param: PN_E0
    // @DisplayName: Intercept LOS-rate blend threshold
    // @Description: Normalized error where LOS-rate guidance becomes fully active. Smaller = more PN (more aggressive) even near center.
    // @Units: -
    // @Range: 0.05 1
    AP_GROUPINFO("PN_E0", 12, ModeIntercept, pn_e0, 0.25f),

    // @Param: THR_NOLK
    // @DisplayName: Intercept fixed throttle when NOT locked
    // @Description: Fixed throttle (0..1) used while INTERCEPT does NOT have a valid target. 0 = disabled (use RC throttle).
    // @Units: -
    // @Range: 0 1
    AP_GROUPINFO("THR_NOLK", 13, ModeIntercept, fixed_throttle_nolock, 0.25f),

    AP_GROUPEND
};

ModeIntercept::ModeIntercept()
{
    AP_Param::setup_object_defaults(this, var_info);
    fwd_i = 0.0f;
    fwd_last_err = 0.0f;
}

bool ModeIntercept::init(bool ignore_checks)
{
    // --- DEBUG: prove mode enters ---
    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "INTC: init called ignore_checks=%u",
                  (unsigned)ignore_checks);

    fwd_i = 0.0f;
    fwd_last_err = 0.0f;

    // Reset internal filters/commands on mode entry
    x_filt = 0.0f;
    y_filt = 0.0f;
    prev_roll_cmd = 0.0f;
    prev_pitch_cmd = 0.0f;
    hold_roll_cmd = 0.0f;
    hold_pitch_cmd = 0.0f;
    x_last_filt = 0.0f;

    // Reset GCS lock/lost state
    intercept_prev_has_target = false;
    intercept_last_valid_target_ms = 0;

    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "INTC: init done");
    return true;
}

void ModeIntercept::run()
{
    // --- DEBUG: prove run() is executing (1Hz) ---
    static uint32_t last_alive_ms = 0;
    const uint32_t now = AP_HAL::millis();
    if (now - last_alive_ms > 1000) {
        last_alive_ms = now;
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                      "INTC: run alive mode=%u armed=%u spool=%u thr0=%u",
                      (unsigned)copter.get_mode(),
                      (unsigned)motors->armed(),
                      (unsigned)motors->get_spool_state(),
                      (unsigned)copter.ap.throttle_zero);
    }

    update_simple_mode();

    // --- Motors / spool state and throttle handling (same pattern as Stabilize) ---
    if (!motors->armed()) {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
    } else if (copter.ap.throttle_zero
               || (copter.air_mode == AirMode::AIRMODE_ENABLED &&
                   motors->get_spool_state() == AP_Motors::SpoolState::SHUT_DOWN)) {

        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
    } else {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    }

    float pilot_desired_throttle = get_pilot_desired_throttle();

    switch (motors->get_spool_state()) {
    case AP_Motors::SpoolState::SHUT_DOWN:
        attitude_control->reset_yaw_target_and_rate();
        attitude_control->reset_rate_controller_I_terms();
        pilot_desired_throttle = 0.0f;
        break;

    case AP_Motors::SpoolState::GROUND_IDLE:
        attitude_control->reset_yaw_target_and_rate();
        attitude_control->reset_rate_controller_I_terms_smoothly();
        pilot_desired_throttle = 0.0f;
        break;

    case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
        if (!motors->limit.throttle_lower) {
            set_land_complete(false);
        }
        break;

    case AP_Motors::SpoolState::SPOOLING_UP:
    case AP_Motors::SpoolState::SPOOLING_DOWN:
        break;
    }

    // --- Yaw: pilot stick rate, like Stabilize ---
    const float target_yaw_rate_rads = get_pilot_desired_yaw_rate_rads();

    // --- Roll/pitch from INTERCEPT target (with smoothing & rate limiting) ---
    float target_roll_rad  = 0.0f;
    float target_pitch_rad = 0.0f;

    const uint32_t timeout_ms = 1000;   // vision message timeout

    // configurable timeout before switching to LOITER
    int32_t lost_ms_param = lost_to_loiter_ms.get();           // INTC_LOST_MS
    lost_ms_param = constrain_int32(lost_ms_param, 0, 10000);
    const uint32_t lost_to_loiter_ms_val = (uint32_t)lost_ms_param;

    bool has_target =
        copter.intercept_target.valid &&
        (copter.intercept_target.quality > 0) &&
        (now - copter.intercept_target.last_update_ms < timeout_ms);

    // --- DEBUG: show what target struct contains (1Hz) ---
    static uint32_t last_tgt_ms = 0;
    if (now - last_tgt_ms > 1000) {
        last_tgt_ms = now;
        const uint32_t age = now - copter.intercept_target.last_update_ms;
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                      "INTC: tgt valid=%u q=%u age=%ums x=%.2f y=%.2f has=%u",
                      (unsigned)copter.intercept_target.valid,
                      (unsigned)copter.intercept_target.quality,
                      (unsigned)age,
                      (double)copter.intercept_target.x_norm,
                      (double)copter.intercept_target.dist_m,
                      (unsigned)has_target);
    }

    // Camera geometry hint (from Pi): quality 200..254 => camera FORWARD (close-in)
    const bool cam_is_forward = has_target &&
        (copter.intercept_target.quality >= 200) &&
        (copter.intercept_target.quality < 255);

    // GCS lock/lost notifications (edge-triggered, broadcast-safe)
    if (has_target && !intercept_prev_has_target) {
        const float x = constrain_float(copter.intercept_target.x_norm, -1.0f, 1.0f);
        const float y = constrain_float(copter.intercept_target.dist_m, -1.0f, 1.0f);
        const uint32_t age = now - copter.intercept_target.last_update_ms;
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                      "INTC: LOCKED x=%.2f y=%.2f age=%ums q=%u",
                      (double)x, (double)y, (unsigned)age,
                      (unsigned)copter.intercept_target.quality);
    } else if (!has_target && intercept_prev_has_target) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "INTC: LOST");
    }
    intercept_prev_has_target = has_target;

    if (has_target) {
        intercept_last_valid_target_ms = now;
    } else {
        if (lost_to_loiter_ms_val > 0 &&
            intercept_last_valid_target_ms != 0 &&
            (now - intercept_last_valid_target_ms) > lost_to_loiter_ms_val) {

            GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                          "INTC: target lost, switching LOITER");
            intercept_last_valid_target_ms = 0;

            set_mode(Mode::Number::LOITER, ModeReason::RC_COMMAND);
            return;
        }
    }

    const bool in_hold_window =
        (!has_target &&
         (intercept_last_valid_target_ms != 0) &&
         (lost_to_loiter_ms_val > 0) &&
         ((now - intercept_last_valid_target_ms) <= lost_to_loiter_ms_val));

    // Max lean angle
    const float max_ang_deg = constrain_float(max_angle_deg.get(), 5.0f, 35.0f);
    const float max_ang_rad = radians(max_ang_deg);

    // dt for PID
    float dt = copter.G_Dt;
    if (!isfinite(dt) || dt <= 0.0f || dt > 0.5f) {
        dt = 0.02f;
    }

    // LPF time constant
    const float tau = 0.30f;
    float alpha = dt / (tau + dt);
    alpha = constrain_float(alpha, 0.0f, 1.0f);

    // Rate limits (deg/s) -> rad/s
    const float max_roll_rate_dps  = 40.0f;
    const float max_pitch_rate_dps = 30.0f;
    const float max_roll_rate_rad  = radians(max_roll_rate_dps);
    const float max_pitch_rate_rad = radians(max_pitch_rate_dps);

    if (has_target) {
        // read + clamp (NOTE: struct field is dist_m, NOT y_norm)
        float x_norm = constrain_float(copter.intercept_target.x_norm, -1.0f, 1.0f);
        float y_norm = constrain_float(copter.intercept_target.dist_m, -1.0f, 1.0f);

        // In forward-camera mode, y_norm is range error to standoff. Negative would mean "too close".
        // Option A: do not reverse away from target; just stop closing.
        if (cam_is_forward && y_norm < 0.0f) {
            y_norm = 0.0f;
        }

        // LPF
        x_filt += alpha * (x_norm - x_filt);
        y_filt += alpha * (y_norm - y_filt);

        // In forward-camera close-in mode, treat small y error as "at standoff" and stop pitching forward
        const float standoff_band = 0.05f;
        const bool at_standoff = cam_is_forward && (y_filt <= standoff_band);

        const float k_lat_g   = k_lat.get();
        const float kp_fwd    = k_fwd.get();
        const float ki_fwd_g  = ki_fwd.get();
        const float kd_fwd_g  = kd_fwd.get();
        const float kd_lat_g  = kd_lat.get();
        const float pn_gain_g = pn_gain.get();
        const float pn_e0_g   = constrain_float(pn_e0.get(), 0.05f, 1.0f);
        const float signx     = (float)(int8_t)sign_x.get();

        // LOS rates (approx): dx/dt
        float dx_dt = 0.0f;
        if (dt > 0.0f) {
            dx_dt = (x_filt - x_last_filt) / dt;
        }
        x_last_filt = x_filt;

        // Blend factor: 0 = pure pursuit, 1 = pure LOS-rate guidance
        const float e_mag = fmaxf(fabsf(x_filt), fabsf(y_filt));
        const float pn_blend = constrain_float(e_mag / pn_e0_g, 0.0f, 1.0f);

        // ---------------- Roll (x axis) ----------------
        const float roll_p  = k_lat_g * x_filt;
        const float roll_pn = pn_gain_g * dx_dt;
        const float roll_d  = kd_lat_g * dx_dt;

        float roll_cmd_raw = signx * ((1.0f - pn_blend) * roll_p + pn_blend * roll_pn + roll_d);

        // ---------------- Pitch (y axis) ----------------
        const float err = at_standoff ? 0.0f : y_filt;

        // Integrator
        if (at_standoff) {
            fwd_i = 0.0f;
        } else if (fabsf(ki_fwd_g) > 1.0e-5f) {
            fwd_i += err * dt;
            const float denom = fmaxf(fabsf(ki_fwd_g), 1.0f);
            const float i_limit = max_ang_rad / denom;
            fwd_i = constrain_float(fwd_i, -i_limit, i_limit);
        } else {
            fwd_i = 0.0f;
        }

        // dy/dt from err
        float derr_dt = 0.0f;
        if (dt > 0.0f) {
            derr_dt = (err - fwd_last_err) / dt;
        }
        fwd_last_err = err;

        float pitch_pursuit = kp_fwd * err + ki_fwd_g * fwd_i;

        const float pitch_pn = pn_gain_g * derr_dt;
        const float pitch_d  = (kd_fwd_g + kd_lat_g) * derr_dt;

        float pid_out = (1.0f - pn_blend) * pitch_pursuit + pn_blend * pitch_pn + pitch_d;

        // sign convention: positive y => forward => negative pitch
        float pitch_cmd_raw = -pid_out;
        if (at_standoff) {
            pitch_cmd_raw = 0.0f;
        }

        // Angle limits BEFORE rate limit
        roll_cmd_raw  = constrain_float(roll_cmd_raw,  -max_ang_rad, max_ang_rad);
        pitch_cmd_raw = constrain_float(pitch_cmd_raw, -max_ang_rad, max_ang_rad);

        // Rate limit
        const float roll_err  = roll_cmd_raw  - prev_roll_cmd;
        const float pitch_err = pitch_cmd_raw - prev_pitch_cmd;

        const float max_roll_step  = max_roll_rate_rad  * dt;
        const float max_pitch_step = max_pitch_rate_rad * dt;

        float roll_cmd  = prev_roll_cmd  + constrain_float(roll_err,  -max_roll_step,  max_roll_step);
        float pitch_cmd = prev_pitch_cmd + constrain_float(pitch_err, -max_pitch_step, max_pitch_step);

        roll_cmd  = constrain_float(roll_cmd,  -max_ang_rad, max_ang_rad);
        pitch_cmd = constrain_float(pitch_cmd, -max_ang_rad, max_ang_rad);

        prev_roll_cmd  = roll_cmd;
        prev_pitch_cmd = pitch_cmd;

        target_roll_rad  = roll_cmd;
        target_pitch_rad = pitch_cmd;

        // update hold
        hold_roll_cmd  = roll_cmd;
        hold_pitch_cmd = pitch_cmd;

    } else if (in_hold_window) {
        // hold last
        target_roll_rad  = hold_roll_cmd;
        target_pitch_rad = hold_pitch_cmd;

    } else {
        // no target
        fwd_i        = 0.0f;
        fwd_last_err = 0.0f;

        x_filt *= (1.0f - alpha);
        y_filt *= (1.0f - alpha);

        // prevent derivative kick
        x_last_filt = x_filt;

        // level with rate limit
        const float roll_cmd_raw  = 0.0f;
        const float pitch_cmd_raw = 0.0f;

        const float roll_err  = roll_cmd_raw  - prev_roll_cmd;
        const float pitch_err = pitch_cmd_raw - prev_pitch_cmd;

        const float max_roll_step  = max_roll_rate_rad  * dt;
        const float max_pitch_step = max_pitch_rate_rad * dt;

        float roll_cmd  = prev_roll_cmd  + constrain_float(roll_err,  -max_roll_step,  max_roll_step);
        float pitch_cmd = prev_pitch_cmd + constrain_float(pitch_err, -max_pitch_step, max_pitch_step);

        roll_cmd  = constrain_float(roll_cmd,  -max_ang_rad, max_ang_rad);
        pitch_cmd = constrain_float(pitch_cmd, -max_ang_rad, max_ang_rad);

        prev_roll_cmd  = roll_cmd;
        prev_pitch_cmd = pitch_cmd;

        target_roll_rad  = roll_cmd;
        target_pitch_rad = pitch_cmd;
    }

    // Attitude control
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_rad(
        target_roll_rad,
        target_pitch_rad,
        target_yaw_rate_rads
    );

    // Throttle: RC or fixed values depending on lock state
    float thr_out = pilot_desired_throttle;

    if (motors->get_spool_state() == AP_Motors::SpoolState::THROTTLE_UNLIMITED) {
        float thr_lock = fixed_throttle.get();        // INTC_THR_MAN
        float thr_nolk = fixed_throttle_nolock.get(); // INTC_THR_NOLK

        thr_lock = constrain_float(thr_lock, 0.0f, 1.0f);
        thr_nolk = constrain_float(thr_nolk, 0.0f, 1.0f);

        if (has_target) {
            if (cam_is_forward) {
                const float standoff_band = 0.05f;
                const bool at_standoff_thr = (y_filt <= standoff_band);
                if (at_standoff_thr) {
                    if (thr_nolk > 0.0f) {
                        thr_out = thr_nolk;
                    }
                } else {
                    if (thr_lock > 0.0f) {
                        thr_out = thr_lock;
                    }
                }
            } else {
                if (thr_lock > 0.0f) {
                    thr_out = thr_lock;
                }
            }
        } else {
            if (thr_nolk > 0.0f) {
                thr_out = thr_nolk;
            }
        }
    }

    attitude_control->set_throttle_out(thr_out, true, g.throttle_filt);
}