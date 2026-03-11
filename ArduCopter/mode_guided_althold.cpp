#include "Copter.h"

// Guided AltHold Mode
// -------------------
// Maintain altitude while allowing guided commands, without GPS.
// In this version we:
//  - Take over roll/pitch/yaw from copter.guided_althold_cmd on entry
//  - Hold altitude with the U controller
//  - Keep the handed-over attitude using the attitude controller
//  - If GPS/EKF become healthy again, switch back to AUTO after a delay

bool ModeGuidedAltHold::init(bool /*ignore_checks*/)
{
    if (!copter.motors->armed()) {
        return false;
    }

    // Initialise vertical (U) controller for altitude hold
    pos_control->init_U_controller();
    pos_control->set_pos_target_U_from_climb_rate_cm(0.0f);
    pos_control->update_U_controller();

    // Hold current heading (no GPS required)
    auto_yaw.set_mode(AutoYaw::Mode::HOLD);

    // Take over attitude from guided_althold_cmd if available,
    // otherwise freeze current AHRS attitude
    if (copter.guided_althold_cmd.valid) {
        _roll_rad  = copter.guided_althold_cmd.roll_rad;
        _pitch_rad = copter.guided_althold_cmd.pitch_rad;
        _yaw_cd    = copter.guided_althold_cmd.yaw_cd;
    } else {
        _roll_rad  = AP::ahrs().get_roll();
        _pitch_rad = AP::ahrs().get_pitch();
        _yaw_cd    = (int32_t)(degrees(AP::ahrs().get_yaw()) * 100.0f);
    }

    // DEBUG: log what Guided AltHold will actually use
    gcs().send_text(
        MAV_SEVERITY_INFO,
        "GAH init: use roll=%.2fdeg pitch=%.2fdeg yaw=%.1fdeg",
        (double)degrees(_roll_rad),
        (double)degrees(_pitch_rad),
        (double)(_yaw_cd * 0.01f)
    );

    // Let motors run normally
    copter.motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    return true;
}

void ModeGuidedAltHold::run()
{
    // Vertical speed/accel limits (m/s, m/s^2)
    const float down_ms   = -get_pilot_speed_dn_ms();          // downward is negative
    const float up_ms     =  (g.pilot_speed_up_cms * 0.01f);
    const float accel_mss =  (g.pilot_accel_u_cmss * 0.01f);
    pos_control->set_max_speed_accel_U_m(down_ms, up_ms, accel_mss);

    // Hold altitude (0 climb), run U controller
    pos_control->set_pos_target_U_from_climb_rate_cm(0.0f);
    pos_control->update_U_controller();

    // Use U-controller thrust
    const Vector3f thrust_vec = pos_control->get_thrust_vector();
    float thrust = thrust_vec.z;   // if your build expects magnitude, change to thrust_vec.length()

    // One-shot debug the first time run() is called after mode entry
    static bool first_run_debug_done = false;
    if (!first_run_debug_done) {
        first_run_debug_done = true;

        gcs().send_text(
            MAV_SEVERITY_INFO,
            "GAH run first: stored roll=%.2fdeg pitch=%.2fdeg yaw=%.1fdeg",
            (double)degrees(_roll_rad),
            (double)degrees(_pitch_rad),
            (double)(_yaw_cd * 0.01f)
        );
    }

    // Feed fixed euler angles (rad) + yaw_cd and thrust to the attitude controller
    // Wrap this call with pragmas to avoid -Wfloat-equal inside the library function.
#if defined(__GNUC__)
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wfloat-equal"
#endif

    attitude_control->input_euler_angle_roll_pitch_yaw_cd(
        _roll_rad,
        _pitch_rad,
        _yaw_cd,
        thrust
    );

#if defined(__GNUC__)
# pragma GCC diagnostic pop
#endif

    // --------------------------------------------------------------------
    // GPS / EKF recovery: if navigation becomes healthy again while we
    // are in GUIDED_ALT_HOLD, switch back to AUTO after a short delay.
    // --------------------------------------------------------------------
    static uint32_t gps_good_since_ms = 0;

    // Simple health checks:
    //  - EKF not in failsafe
    //  - GPS has at least 3D fix and some satellites
    const bool ekf_healthy = !copter.failsafe.ekf;
    const bool gps_good =
        (AP::gps().status() >= AP_GPS::GPS_OK_FIX_3D) &&
        (AP::gps().num_sats() >= 8);

    if (ekf_healthy && gps_good) {
        const uint32_t now = AP_HAL::millis();
        if (gps_good_since_ms == 0) {
            gps_good_since_ms = now;
        } else if (now - gps_good_since_ms > 5000U) {  // 5 seconds stable
            gcs().send_text(
                MAV_SEVERITY_INFO,
                "GAH: GPS/EKF recovered, switching back to AUTO"
            );
            // Use a reason that definitely exists in your ModeReason enum
            copter.set_mode(Mode::Number::AUTO, ModeReason::GCS_COMMAND);
            gps_good_since_ms = 0;
            return;    // hand control back to AUTO
        }
    } else {
        // Any time GPS or EKF are not good, reset the timer
        gps_good_since_ms = 0;
    }
}

void ModeGuidedAltHold::set_pilot_desired_attitude(float roll_cd, float pitch_cd, float yaw_cd)
{
    _roll_rad  = radians(roll_cd  / 100.0f);
    _pitch_rad = radians(pitch_cd / 100.0f);
    _yaw_cd    = yaw_cd;

    // DEBUG: log what attitude was handed over to this mode
    gcs().send_text(
        MAV_SEVERITY_INFO,
        "GAH set_pilot: roll=%.2fdeg pitch=%.2fdeg yaw=%.1fdeg",
        (double)(roll_cd  / 100.0f),
        (double)(pitch_cd / 100.0f),
        (double)(yaw_cd   / 100.0f)
    );

    gcs().send_text(
        MAV_SEVERITY_INFO,
        "GAH set_pilot raw: roll_rad=%.3f pitch_rad=%.3f yaw_cd=%ld",
        (double)_roll_rad,
        (double)_pitch_rad,
        (long)_yaw_cd
    );

    // If you want to allow pilot yaw changes during this mode:
    // auto_yaw.set_fixed_yaw_cd(_yaw_cd);
}

bool ModeGuidedAltHold::is_ready()
{
    return copter.motors->armed();
}

bool ModeGuidedAltHold::requires_GPS() const
{
    return false;
}
