#include "Copter.h"

#if AC_PRECLAND_ENABLED
bool Mode::_cached_yaw_allow_descent = false;
bool Mode::_cached_yaw_aligned = false;
float Mode::_cached_desired_yaw_rad = 0.0f;
float Mode::_cached_yaw_error_deg = 0.0f;
uint8_t Mode::_cached_yaw_state = static_cast<uint8_t>(AC_PrecLand::YawAlignState::SEARCHING);
uint32_t Mode::_yaw_cache_update_ms = 0;
// Static variables for cache control
static uint32_t _last_control_cycle_ms = 0;
static bool _cache_valid_this_cycle = false;
#endif

/*
 * High level calls to set and update flight modes logic for individual
 * flight modes is in control_acro.cpp, control_stabilize.cpp, etc
 */

/*
  constructor for Mode object
 */
Mode::Mode(void) :
    g(copter.g),
    g2(copter.g2),
    wp_nav(copter.wp_nav),
    loiter_nav(copter.loiter_nav),
    pos_control(copter.pos_control),
    ahrs(copter.ahrs),
    attitude_control(copter.attitude_control),
    motors(copter.motors),
    channel_roll(copter.channel_roll),
    channel_pitch(copter.channel_pitch),
    channel_throttle(copter.channel_throttle),
    channel_yaw(copter.channel_yaw),
    G_Dt(copter.G_Dt)
{ };

#if AC_PAYLOAD_PLACE_ENABLED
PayloadPlace Mode::payload_place;
#endif

// return the static controller object corresponding to supplied mode
Mode *Copter::mode_from_mode_num(const Mode::Number mode)
{

    switch (mode) {
#if MODE_ACRO_ENABLED
        case Mode::Number::ACRO:
            return &mode_acro;
#endif

        case Mode::Number::STABILIZE:
            return &mode_stabilize;

        case Mode::Number::ALT_HOLD:
            return &mode_althold;

#if MODE_AUTO_ENABLED
        case Mode::Number::AUTO:
            return &mode_auto;
#endif

#if MODE_CIRCLE_ENABLED
        case Mode::Number::CIRCLE:
            return &mode_circle;
#endif

#if MODE_LOITER_ENABLED
        case Mode::Number::LOITER:
            return &mode_loiter;
#endif

#if MODE_GUIDED_ENABLED
        case Mode::Number::GUIDED:
            return &mode_guided;
#endif

        case Mode::Number::LAND:
            return &mode_land;

#if MODE_RTL_ENABLED
        case Mode::Number::RTL:
            return &mode_rtl;
#endif

#if MODE_DRIFT_ENABLED
        case Mode::Number::DRIFT:
            return &mode_drift;
#endif

#if MODE_SPORT_ENABLED
        case Mode::Number::SPORT:
            return &mode_sport;
#endif

#if MODE_FLIP_ENABLED
        case Mode::Number::FLIP:
            return &mode_flip;
#endif

#if AUTOTUNE_ENABLED
        case Mode::Number::AUTOTUNE:
            return &mode_autotune;
#endif

#if MODE_POSHOLD_ENABLED
        case Mode::Number::POSHOLD:
            return &mode_poshold;
#endif

#if MODE_BRAKE_ENABLED
        case Mode::Number::BRAKE:
            return &mode_brake;
#endif

#if MODE_THROW_ENABLED
        case Mode::Number::THROW:
            return &mode_throw;
#endif

#if AP_ADSB_AVOIDANCE_ENABLED
        case Mode::Number::AVOID_ADSB:
            return &mode_avoid_adsb;
#endif

#if MODE_GUIDED_NOGPS_ENABLED
        case Mode::Number::GUIDED_NOGPS:
            return &mode_guided_nogps;
#endif

#if MODE_SMARTRTL_ENABLED
        case Mode::Number::SMART_RTL:
            return &mode_smartrtl;
#endif

#if MODE_FLOWHOLD_ENABLED
        case Mode::Number::FLOWHOLD:
            return (Mode *)g2.mode_flowhold_ptr;
#endif

#if MODE_FOLLOW_ENABLED
        case Mode::Number::FOLLOW:
            return &mode_follow;
#endif

#if MODE_ZIGZAG_ENABLED
        case Mode::Number::ZIGZAG:
            return &mode_zigzag;
#endif

#if MODE_SYSTEMID_ENABLED
        case Mode::Number::SYSTEMID:
            return (Mode *)g2.mode_systemid_ptr;
#endif

#if MODE_AUTOROTATE_ENABLED
        case Mode::Number::AUTOROTATE:
            return &mode_autorotate;
#endif

#if MODE_TURTLE_ENABLED
        case Mode::Number::TURTLE:
            return &mode_turtle;
#endif

        default:
            break;
    }

#if MODE_GUIDED_ENABLED && AP_SCRIPTING_ENABLED
    // Check registered custom modes
    for (uint8_t i = 0; i < ARRAY_SIZE(mode_guided_custom); i++) {
        if ((mode_guided_custom[i] != nullptr) && (mode_guided_custom[i]->mode_number() == mode)) {
            return mode_guided_custom[i];
        }
    }
#endif

    return nullptr;
}


// called when an attempt to change into a mode is unsuccessful:
void Copter::mode_change_failed(const Mode *mode, const char *reason)
{
    gcs().send_text(MAV_SEVERITY_WARNING, "Mode change to %s failed: %s", mode->name(), reason);
    LOGGER_WRITE_ERROR(LogErrorSubsystem::FLIGHT_MODE, LogErrorCode(mode->mode_number()));
    // make sad noise
    if (copter.ap.initialised) {
        AP_Notify::events.user_mode_change_failed = 1;
    }
}

// Check if this mode can be entered from the GCS
bool Copter::gcs_mode_enabled(const Mode::Number mode_num)
{
    // List of modes that can be blocked, index is bit number in parameter bitmask
    static const uint8_t mode_list [] {
        (uint8_t)Mode::Number::STABILIZE,
        (uint8_t)Mode::Number::ACRO,
        (uint8_t)Mode::Number::ALT_HOLD,
        (uint8_t)Mode::Number::AUTO,
        (uint8_t)Mode::Number::GUIDED,
        (uint8_t)Mode::Number::LOITER,
        (uint8_t)Mode::Number::CIRCLE,
        (uint8_t)Mode::Number::DRIFT,
        (uint8_t)Mode::Number::SPORT,
        (uint8_t)Mode::Number::FLIP,
        (uint8_t)Mode::Number::AUTOTUNE,
        (uint8_t)Mode::Number::POSHOLD,
        (uint8_t)Mode::Number::BRAKE,
        (uint8_t)Mode::Number::THROW,
        (uint8_t)Mode::Number::AVOID_ADSB,
        (uint8_t)Mode::Number::GUIDED_NOGPS,
        (uint8_t)Mode::Number::SMART_RTL,
        (uint8_t)Mode::Number::FLOWHOLD,
        (uint8_t)Mode::Number::FOLLOW,
        (uint8_t)Mode::Number::ZIGZAG,
        (uint8_t)Mode::Number::SYSTEMID,
        (uint8_t)Mode::Number::AUTOROTATE,
        (uint8_t)Mode::Number::AUTO_RTL,
        (uint8_t)Mode::Number::TURTLE
    };

    if (!block_GCS_mode_change((uint8_t)mode_num, mode_list, ARRAY_SIZE(mode_list))) {
        return true;
    }

    // Mode disabled, try and grab a mode name to give a better warning.
    Mode *new_flightmode = mode_from_mode_num(mode_num);
    if (new_flightmode != nullptr) {
        mode_change_failed(new_flightmode, "GCS entry disabled (FLTMODE_GCSBLOCK)");
    } else {
        notify_no_such_mode((uint8_t)mode_num);
    }

    return false;
}

// set_mode - change flight mode and perform any necessary initialisation
// optional force parameter used to force the flight mode change (used only first time mode is set)
// returns true if mode was successfully set
// ACRO, STABILIZE, ALTHOLD, LAND, DRIFT and SPORT can always be set successfully but the return state of other flight modes should be checked and the caller should deal with failures appropriately
bool Copter::set_mode(Mode::Number mode, ModeReason reason)
{
    // update last reason
    const ModeReason last_reason = _last_reason;
    _last_reason = reason;

    // return immediately if we are already in the desired mode
    if (mode == flightmode->mode_number()) {
        control_mode_reason = reason;
        // set yaw rate time constant during autopilot startup
        if (reason == ModeReason::INITIALISED && mode == Mode::Number::STABILIZE) {
            attitude_control->set_yaw_rate_tc(g2.command_model_pilot_y.get_rate_tc());
        }
        // make happy noise
        if (copter.ap.initialised && (reason != last_reason)) {
            AP_Notify::events.user_mode_change = 1;
        }
        return true;
    }

    // Check if GCS mode change is disabled via parameter
    if ((reason == ModeReason::GCS_COMMAND) && !gcs_mode_enabled(mode)) {
        return false;
    }

#if MODE_AUTO_ENABLED
    if (mode == Mode::Number::AUTO_RTL) {
        // Special case for AUTO RTL, not a true mode, just AUTO in disguise
        // Attempt to join return path, fallback to do-land-start
        return mode_auto.return_path_or_jump_to_landing_sequence_auto_RTL(reason);
    }
#endif

    Mode *new_flightmode = mode_from_mode_num(mode);
    if (new_flightmode == nullptr) {
        notify_no_such_mode((uint8_t)mode);
        return false;
    }

    bool ignore_checks = !motors->armed();   // allow switching to any mode if disarmed.  We rely on the arming check to perform

#if FRAME_CONFIG == HELI_FRAME
    // do not allow helis to enter a non-manual throttle mode if the
    // rotor runup is not complete
    if (!ignore_checks && !new_flightmode->has_manual_throttle() && !motors->rotor_runup_complete()) {
        mode_change_failed(new_flightmode, "runup not complete");
        return false;
    }
#endif

#if FRAME_CONFIG != HELI_FRAME
    // ensure vehicle doesn't leap off the ground if a user switches
    // into a manual throttle mode from a non-manual-throttle mode
    // (e.g. user arms in guided, raises throttle to 1300 (not enough to
    // trigger auto takeoff), then switches into manual):
    bool user_throttle = new_flightmode->has_manual_throttle();
#if MODE_DRIFT_ENABLED
    if (new_flightmode == &mode_drift) {
        user_throttle = true;
    }
#endif
    if (!ignore_checks &&
        ap.land_complete &&
        user_throttle &&
        !copter.flightmode->has_manual_throttle() &&
        new_flightmode->get_pilot_desired_throttle() > copter.get_non_takeoff_throttle()) {
        mode_change_failed(new_flightmode, "throttle too high");
        return false;
    }
#endif

    if (!ignore_checks &&
        new_flightmode->requires_GPS() &&
        !copter.position_ok()) {
        mode_change_failed(new_flightmode, "requires position");
        return false;
    }

    // check for valid altitude if old mode did not require it but new one does
    // we only want to stop changing modes if it could make things worse
    if (!ignore_checks &&
        !copter.ekf_alt_ok() &&
        flightmode->has_manual_throttle() &&
        !new_flightmode->has_manual_throttle()) {
        mode_change_failed(new_flightmode, "need alt estimate");
        return false;
    }

#if AP_FENCE_ENABLED
    // may not be allowed to change mode if recovering from fence breach
    if (!ignore_checks &&
        fence.enabled() &&
        fence.option_enabled(AC_Fence::OPTIONS::DISABLE_MODE_CHANGE) &&
        fence.get_breaches() &&
        motors->armed() &&
        get_control_mode_reason() == ModeReason::FENCE_BREACHED &&
        !ap.land_complete) {
        mode_change_failed(new_flightmode, "in fence recovery");
        return false;
    }
#endif

    if (rc().in_rc_failsafe() && !new_flightmode->allows_entry_in_rc_failsafe()) {
        mode_change_failed(new_flightmode, "in RC failsafe");
        return false;
    }

    if (!new_flightmode->init(ignore_checks)) {
        mode_change_failed(new_flightmode, "init failed");
        return false;
    }

    // perform any cleanup required by previous flight mode
    exit_mode(flightmode, new_flightmode);

    // update flight mode
    flightmode = new_flightmode;
    control_mode_reason = reason;
#if HAL_LOGGING_ENABLED
    logger.Write_Mode((uint8_t)flightmode->mode_number(), reason);
#endif
    gcs().send_message(MSG_HEARTBEAT);

#if HAL_ADSB_ENABLED
    adsb.set_is_auto_mode((mode == Mode::Number::AUTO) || (mode == Mode::Number::RTL) || (mode == Mode::Number::GUIDED));
#endif

#if AP_FENCE_ENABLED
    if (fence.get_action() != AC_Fence::Action::REPORT_ONLY) {
        // pilot requested flight mode change during a fence breach indicates pilot is attempting to manually recover
        // this flight mode change could be automatic (i.e. fence, battery, GPS or GCS failsafe)
        // but it should be harmless to disable the fence temporarily in these situations as well
        fence.manual_recovery_start();
    }
#endif

#if AP_CAMERA_ENABLED
    camera.set_is_auto_mode(flightmode->mode_number() == Mode::Number::AUTO);
#endif

    // set rate shaping time constants
#if MODE_ACRO_ENABLED || MODE_SPORT_ENABLED
    attitude_control->set_roll_pitch_rate_tc(g2.command_model_acro_rp.get_rate_tc());
#endif
    attitude_control->set_yaw_rate_tc(g2.command_model_pilot_y.get_rate_tc());
#if MODE_ACRO_ENABLED || MODE_DRIFT_ENABLED
    if (mode== Mode::Number::ACRO || mode== Mode::Number::DRIFT) {
        attitude_control->set_yaw_rate_tc(g2.command_model_acro_y.get_rate_tc());
    }
#endif

    // update notify object
    notify_flight_mode();

    // make happy noise
    if (copter.ap.initialised) {
        AP_Notify::events.user_mode_change = 1;
    }

    // return success
    return true;
}

bool Copter::set_mode(const uint8_t new_mode, const ModeReason reason)
{
    static_assert(sizeof(Mode::Number) == sizeof(new_mode), "The new mode can't be mapped to the vehicles mode number");
#ifdef DISALLOW_GCS_MODE_CHANGE_DURING_RC_FAILSAFE
    if (reason == ModeReason::GCS_COMMAND && copter.failsafe.radio) {
        // don't allow mode changes while in radio failsafe
        return false;
    }
#endif
    return copter.set_mode(static_cast<Mode::Number>(new_mode), reason);
}

// update_flight_mode - calls the appropriate attitude controllers based on flight mode
// called at 100hz or more
void Copter::update_flight_mode()
{
#if AP_RANGEFINDER_ENABLED
    surface_tracking.invalidate_for_logging();  // invalidate surface tracking alt, flight mode will set to true if used
#endif
    attitude_control->landed_gain_reduction(copter.ap.land_complete); // Adjust gains when landed to attenuate ground oscillation

    // set ekf reset handling method
    pos_control->set_reset_handling_method(flightmode->move_vehicle_on_ekf_reset() ? AC_PosControl::EKFResetMethod::MoveVehicle : AC_PosControl::EKFResetMethod::MoveTarget);

    flightmode->run();
}

// exit_mode - high level call to organise cleanup as a flight mode is exited
void Copter::exit_mode(Mode *&old_flightmode,
                       Mode *&new_flightmode)
{
    // smooth throttle transition when switching from manual to automatic flight modes
    if (old_flightmode->has_manual_throttle() && !new_flightmode->has_manual_throttle() && motors->armed() && !ap.land_complete) {
        // this assumes all manual flight modes use get_pilot_desired_throttle to translate pilot input to output throttle
        set_accel_throttle_I_from_pilot_throttle();
    }

    // cancel any takeoffs in progress
    old_flightmode->takeoff_stop();

    // perform cleanup required for each flight mode
    old_flightmode->exit();

#if FRAME_CONFIG == HELI_FRAME
    // firmly reset the flybar passthrough to false when exiting acro mode.
    if (old_flightmode == &mode_acro) {
        attitude_control->use_flybar_passthrough(false, false);
        motors->set_acro_tail(false);
    }

    //last collective output
    input_manager.set_last_coll_output(motors->get_throttle());

    // if we are changing from a mode that did not use manual throttle,
    // collective ramp functions should be called to blend the transition
    if (new_flightmode->has_manual_throttle()) {
        input_manager.set_collective_ramp(1.0);
    }

    // Make sure inverted flight is disabled if not supported in the new mode
    if (!new_flightmode->allows_inverted()) {
        attitude_control->set_inverted_flight(false);
    }
#endif //HELI_FRAME
}

// notify_flight_mode - sets notify object based on current flight mode.  Only used for OreoLED notify device
void Copter::notify_flight_mode() {
    AP_Notify::flags.autopilot_mode = flightmode->is_autopilot();
    AP_Notify::flags.flight_mode = (uint8_t)flightmode->mode_number();
    notify.set_flight_mode_str(flightmode->name4());
}

// get_pilot_desired_angle - transform pilot's roll or pitch input into a desired lean angle
// returns desired angle in radians
void Mode::get_pilot_desired_lean_angles_rad(float &roll_out_rad, float &pitch_out_rad, float angle_max_rad, float angle_limit_rad) const
{
    // throttle failsafe check
    if (!rc().has_valid_input()) {
        roll_out_rad = 0.0;
        pitch_out_rad = 0.0;
        return;
    }

    //transform pilot's normalised roll or pitch stick input into a roll and pitch euler angle command
    rc_input_to_roll_pitch_rad(channel_roll->norm_input_dz(), channel_pitch->norm_input_dz(), angle_max_rad,  angle_limit_rad, roll_out_rad, pitch_out_rad);
}

// transform pilot's roll or pitch input into a desired velocity
Vector2f Mode::get_pilot_desired_velocity(float vel_max) const
{
    Vector2f vel;

    if (!rc().has_valid_input()) {
        return vel;
    }
    // fetch roll and pitch inputs
    float roll_out = channel_roll->norm_input_dz();
    float pitch_out = channel_pitch->norm_input_dz();

    // convert roll and pitch inputs into velocity in NE frame
    vel = Vector2f(-pitch_out, roll_out);
    if (vel.is_zero()) {
        return vel;
    }
    vel = copter.ahrs.body_to_earth2D(vel);

    // Transform square input range to circular output
    // vel_scalar is the vector to the edge of the +- 1.0 square in the direction of the current input
    Vector2f vel_scalar = vel / MAX(fabsf(vel.x), fabsf(vel.y));
    // We scale the output by the ratio of the distance to the square to the unit circle and multiply by vel_max
    vel *= vel_max / vel_scalar.length();
    return vel;
}

bool Mode::_TakeOff::triggered_ms(const float target_climb_rate_ms) const
{
    if (!copter.ap.land_complete) {
        // can't take off if we're already flying
        return false;
    }
    if (target_climb_rate_ms <= 0.0f) {
        // can't takeoff unless we want to go up...
        return false;
    }

    if (copter.motors->get_spool_state() != AP_Motors::SpoolState::THROTTLE_UNLIMITED) {
        // hold aircraft on the ground until rotor speed runup has finished
        return false;
    }

    return true;
}

bool Mode::is_disarmed_or_landed() const
{
    if (!motors->armed() || !copter.ap.auto_armed || copter.ap.land_complete) {
        return true;
    }
    return false;
}

void Mode::zero_throttle_and_relax_ac(bool spool_up)
{
    if (spool_up) {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    } else {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
    }
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_rad(0.0f, 0.0f, 0.0f);
    attitude_control->set_throttle_out(0.0f, false, copter.g.throttle_filt);
}

void Mode::zero_throttle_and_hold_attitude()
{
    // run attitude controller
    attitude_control->input_rate_bf_roll_pitch_yaw_rads(0.0f, 0.0f, 0.0f);
    attitude_control->set_throttle_out(0.0f, false, copter.g.throttle_filt);
}

// handle situations where the vehicle is on the ground waiting for takeoff
// force_throttle_unlimited should be true in cases where we want to keep the motors spooled up
// (instead of spooling down to ground idle).  This is required for tradheli's in Guided and Auto
// where we always want the motor spooled up in Guided or Auto mode.  Tradheli's main rotor stops 
// when spooled down to ground idle.
// ultimately it forces the motor interlock to be obeyed in auto and guided modes when on the ground.
void Mode::make_safe_ground_handling(bool force_throttle_unlimited)
{
    if (force_throttle_unlimited) {
        // keep rotors turning 
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    } else {
        // spool down to ground idle
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
    }

    // aircraft is landed, integrator terms must be reset regardless of spool state
    attitude_control->reset_rate_controller_I_terms_smoothly();
 
    switch (motors->get_spool_state()) {
    case AP_Motors::SpoolState::SHUT_DOWN:
    case AP_Motors::SpoolState::GROUND_IDLE:
        // reset yaw targets and rates during idle states
        attitude_control->reset_yaw_target_and_rate();
        break;
    case AP_Motors::SpoolState::SPOOLING_UP:
    case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
    case AP_Motors::SpoolState::SPOOLING_DOWN:
        // while transitioning though active states continue to operate normally
        break;
    }

    pos_control->NE_relax_velocity_controller();
    pos_control->NE_update_controller();
    pos_control->D_relax_controller(0.0f);   // forces throttle output to decay to zero
    pos_control->D_update_controller();
    // we may need to move this out
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_rad(0.0f, 0.0f, 0.0f);
}

/*
  get a height above ground estimate for landing
 */
float Mode::get_alt_above_ground_m(void) const
{
    float alt_above_ground_m;
    if (copter.get_rangefinder_height_interpolated_m(alt_above_ground_m)) {
        return alt_above_ground_m;
    }
    if (!copter.current_loc.initialised()) {
        // current loc uninitialised during startup, return zero
        return 0;
    }
    if (copter.current_loc.get_alt_m(Location::AltFrame::ABOVE_TERRAIN, alt_above_ground_m)) {
        return alt_above_ground_m;
    }

    // Assume the Earth is flat:
    return copter.current_loc.alt * 0.01;
}

void Mode::land_run_vertical_control(bool pause_descent)
{
    float climb_rate_ms = 0;
    bool ignore_descent_limit = false;
    
#if AC_PRECLAND_ENABLED
    // =================================================================
    // KRITISCH: Cache MUSS zuerst aktualisiert werden!
    // =================================================================
    if (copter.precland.yaw_align_enabled()) {
        // Cache explizit aktualisieren bevor wir ihn lesen
        update_yaw_align_cache();
        
        uint8_t state = get_cached_yaw_state();
        
        // Pause descent during active alignment (independent of XY error)
        if (state == static_cast<uint8_t>(AC_PrecLand::YawAlignState::COARSE_ALIGNING) ||
            state == static_cast<uint8_t>(AC_PrecLand::YawAlignState::FINE_ALIGNING)) {
            pause_descent = true;
        }
        
        // Use cached value as additional safety check
        if (!get_cached_yaw_allow_descent()) {
            pause_descent = true;
        }
    }
#endif
    
    if (!pause_descent) {
        // do not ignore limits until we have slowed down for landing
        ignore_descent_limit = (MAX(g2.land_alt_low_cm, 100) * 0.01 > get_alt_above_ground_m()) || copter.ap.land_complete_maybe;

        float max_land_descent_speed_ms;
        if (g.land_speed_high_cms > 0) {
            max_land_descent_speed_ms = g.land_speed_high_cms * 0.01;
        } else {
            max_land_descent_speed_ms = pos_control->get_max_speed_down_ms();
        }

        // Don't speed up for landing.
        max_land_descent_speed_ms = MAX(max_land_descent_speed_ms, abs(g.land_speed_cms) * 0.01);

        // Compute a vertical velocity demand
        climb_rate_ms = sqrt_controller(MAX(g2.land_alt_low_cm, 100) * 0.01 - get_alt_above_ground_m(), pos_control->D_get_pos_p().kP(), pos_control->D_get_max_accel_mss(), G_Dt);

        // Constrain the demanded vertical velocity
        climb_rate_ms = constrain_float(climb_rate_ms, -max_land_descent_speed_ms, -abs(g.land_speed_cms) * 0.01);

#if AC_PRECLAND_ENABLED
        const bool navigating = pos_control->NE_is_active();
        // FIX: Use target_visible() for immediate reaction to backend measurements
        // This allows descent control to react immediately without waiting for EKF initialization
        bool target_is_visible = !copter.ap.land_repo_active && copter.precland.target_visible() && navigating;
        bool doing_precision_landing = !copter.ap.land_repo_active && copter.precland.target_acquired() && navigating;
        
        // FIX: Check if yaw alignment is complete - if so, allow descent even if target is temporarily lost
        bool yaw_alignment_complete = copter.precland.yaw_align_enabled() && 
            (get_cached_yaw_state() == static_cast<uint8_t>(AC_PrecLand::YawAlignState::DESCENDING) ||
             get_cached_yaw_state() == static_cast<uint8_t>(AC_PrecLand::YawAlignState::FINAL_DESCENT));

        // Immediately pause descent when target becomes visible (even before EKF initialization)
        // BUT: Allow descent if yaw alignment is complete (even if target temporarily lost)
        if (target_is_visible && !doing_precision_landing && !yaw_alignment_complete) {
            // Target is visible but EKF not yet initialized - pause descent
            // BUT only if alignment is not complete yet
            climb_rate_ms = 0.0f;
        } else if (doing_precision_landing || yaw_alignment_complete) {
            Vector2p target_pos_ne_m;
            float target_error_m = 0.0f;
            bool have_target_pos = copter.precland.get_target_position_m(target_pos_ne_m);
            
            // FIX: If alignment complete but target lost, use last known position
            if (!have_target_pos && yaw_alignment_complete) {
                Vector3p last_target_pos_ned;
                copter.precland.get_last_detected_landing_pos_NED_m(last_target_pos_ned);
                target_pos_ne_m = last_target_pos_ned.xy();
                // Set error to 0 since we're using last known position (assume we're there)
                target_error_m = 0.0f;
            } else if (have_target_pos) {
                const Vector2p current_pos_ne_m = pos_control->get_pos_estimate_NED_m().xy();
                target_error_m = (target_pos_ne_m - current_pos_ne_m).tofloat().length();
            }
            
            const float max_horiz_pos_error_m = copter.precland.get_max_xy_error_before_descending_m();
            Vector3f target_pos_meas_ned_m;
            copter.precland.get_target_position_measurement_NED_m(target_pos_meas_ned_m);
            
            // FIX: After alignment, allow descent even if target temporarily lost
            // Only check XY error if we have valid target position
            if (have_target_pos && target_error_m > max_horiz_pos_error_m && !is_zero(max_horiz_pos_error_m)) {
                climb_rate_ms = 0.0f;
            } else if (target_pos_meas_ned_m.z > 0.35 && target_pos_meas_ned_m.z < 2.0 && !copter.precland.do_fast_descend() && have_target_pos) {
                // Verwende Parameter statt Konstanten
                const float precland_acceptable_error_m = copter.precland.get_acceptable_error_m();
                const float precland_min_descent_speed_ms = copter.precland.get_min_descent_speed_ms();
                const float max_descent_speed_ms = abs(g.land_speed_cms) * 0.005;
                const float land_slowdown_ms = MAX(0.0f, target_error_m * (max_descent_speed_ms / precland_acceptable_error_m));
                climb_rate_ms = MIN(-precland_min_descent_speed_ms, -max_descent_speed_ms + land_slowdown_ms);
            }
            // FIX: If alignment complete but no target measurement, use normal descent rate
            // This allows descent to continue even if target is temporarily lost
        }
#endif
    }

    pos_control->D_set_pos_target_from_climb_rate_ms(climb_rate_ms, ignore_descent_limit);
    pos_control->D_update_controller();
}

#if AC_PRECLAND_ENABLED

void Mode::update_yaw_align_cache()
{
    const uint32_t now_ms = AP_HAL::millis();
    
    // Only update once per control cycle
    // But: if cache was invalidated, ALWAYS update
    if (_cache_valid_this_cycle && (now_ms - _last_control_cycle_ms) < 3) {
        return;
    }
    
    // Neuer Control-Zyklus beginnt
    _last_control_cycle_ms = now_ms;
    _cache_valid_this_cycle = true;
    
    // Get rangefinder altitude
    float rangefinder_alt_m = -1.0f;
    if (copter.rangefinder_state.enabled && copter.rangefinder_state.alt_healthy) {
        rangefinder_alt_m = copter.rangefinder_state.alt_m;
    }
    
    // State Machine EINMAL aktualisieren
    AC_PrecLand::YawAlignResult result = copter.precland.yaw_align_update(rangefinder_alt_m);
    
    // Ergebnisse cachen
    _cached_yaw_allow_descent = result.allow_descent;
    _cached_yaw_aligned = result.yaw_aligned;
    _cached_desired_yaw_rad = result.desired_yaw_rad;
    _cached_yaw_error_deg = result.yaw_error_deg;
    _cached_yaw_state = static_cast<uint8_t>(result.state);
    _yaw_cache_update_ms = now_ms;
}

// Getter mit automatischem Cache-Update
bool Mode::get_cached_yaw_allow_descent() 
{
    update_yaw_align_cache();
    return _cached_yaw_allow_descent;
}

float Mode::get_cached_desired_yaw_rad()
{
    update_yaw_align_cache();
    return _cached_desired_yaw_rad;
}

uint8_t Mode::get_cached_yaw_state()
{
    update_yaw_align_cache();
    return _cached_yaw_state;
}

void Mode::invalidate_yaw_align_cache()
{
    _cache_valid_this_cycle = false;
    // Also set default values to safe state
    _cached_yaw_allow_descent = false;  // Safe: descent not allowed
    _cached_yaw_state = static_cast<uint8_t>(AC_PrecLand::YawAlignState::SEARCHING);
}
#endif  // AC_PRECLAND_ENABLED 

void Mode::land_run_horizontal_control()
{
    Vector2f vel_correction_ms;
    
    // STEP 1: Consider yaw alignment state
    copter.ap.prec_land_active = false;
#if AC_PRECLAND_ENABLED
    // FIX: Also activate when target is visible (even before EKF initialization)
    // This ensures position control is active immediately when target is detected
    bool target_is_visible = copter.precland.target_visible();
    bool target_is_acquired = copter.precland.target_acquired();
    AC_PrecLand::YawAlignState yaw_state = copter.precland.get_yaw_align_state();
    
    // FIX: SIMPLIFIED LOGIC - Keep prec_land_active true whenever yaw alignment is active
    // This prevents forward movement even when target is temporarily lost
    // Once yaw alignment starts (any state except DISABLED), we want position control active
    bool yaw_align_active = copter.precland.yaw_align_enabled() && 
        (yaw_state != AC_PrecLand::YawAlignState::DISABLED);
    
    copter.ap.prec_land_active = !copter.ap.land_repo_active && 
        (target_is_acquired || target_is_visible || yaw_align_active);
#endif

    // ========================================================================
    // STEP 2: Process pilot inputs (may change land_repo_active)
    // ========================================================================
    if (rc().has_valid_input()) {
        if ((g.throttle_behavior & THR_BEHAVE_HIGH_THROTTLE_CANCELS_LAND) != 0 && 
            copter.rc_throttle_control_in_filter.get() > LAND_CANCEL_TRIGGER_THR) {
            LOGGER_WRITE_EVENT(LogEvent::LAND_CANCELLED_BY_PILOT);
            if (!set_mode(Mode::Number::LOITER, ModeReason::THROTTLE_LAND_ESCAPE)) {
                set_mode(Mode::Number::ALT_HOLD, ModeReason::THROTTLE_LAND_ESCAPE);
            }
            return;
        }

        if (g.land_repositioning) {
            update_simple_mode();
            const float max_pilot_vel_ms = wp_nav->get_wp_acceleration_mss() * 0.5f;
            vel_correction_ms = get_pilot_desired_velocity(max_pilot_vel_ms);

            if (!vel_correction_ms.is_zero()) {
                if (!copter.ap.land_repo_active) {
                    LOGGER_WRITE_EVENT(LogEvent::LAND_REPO_ACTIVE);
                }
                copter.ap.land_repo_active = true;
#if AC_PRECLAND_ENABLED
            } else {
                if (copter.precland.allow_precland_after_reposition()) {
                    copter.ap.land_repo_active = false;
                }
#endif
            }
        }
    }

    // ========================================================================
    // STEP 3: Re-evaluate prec_land_active AFTER pilot input processing
    // ========================================================================
#if AC_PRECLAND_ENABLED
    // Re-evaluate after pilot input processing (variables already declared in STEP 1)
    target_is_visible = copter.precland.target_visible();
    target_is_acquired = copter.precland.target_acquired();
    yaw_state = copter.precland.get_yaw_align_state();
    
    // FIX: SIMPLIFIED LOGIC - Keep prec_land_active true whenever yaw alignment is active
    yaw_align_active = copter.precland.yaw_align_enabled() && 
        (yaw_state != AC_PrecLand::YawAlignState::DISABLED);
    
    copter.ap.prec_land_active = !copter.ap.land_repo_active && 
        (target_is_acquired || target_is_visible || yaw_align_active);
#endif

    // ========================================================================
    // STEP 4: Relax loiter target if we might be landed
    // ========================================================================
    if (copter.ap.land_complete_maybe) {
        pos_control->NE_soften_for_landing();
    }

    // ========================================================================
    // STEP 5: Position control based on prec_land_active
    // ========================================================================
#if AC_PRECLAND_ENABLED
    if (copter.ap.prec_land_active) {
        Vector2p target_pos_ne_m;
        Vector2f target_vel_ne_ms;
        
        // FIX: During yaw alignment, the target position estimate is UNRELIABLE
        // because the camera may be pointing in a different direction.
        // Only use target position during DESCENDING (active correction).
        // In FINAL_DESCENT, just hold position and go straight down - the drone
        // should already be well-centered from fine alignment, and any horizontal
        // movement at low altitude risks losing the target.
        bool alignment_complete = (yaw_state == AC_PrecLand::YawAlignState::DESCENDING);
        
        // Check if we have a valid target position (only trust it if alignment is complete)
        bool have_valid_target_pos = false;
        if (alignment_complete) {
            // After alignment, use target position for precision landing
            have_valid_target_pos = copter.precland.get_target_position_m(target_pos_ne_m);
            
            // SANITY CHECK: Verify the target position measurement makes sense
            // Now that we use rangefinder for distance, meas_alt should be close to RF alt
            // Only reject if they're DRASTICALLY different (factor of 5+)
            if (have_valid_target_pos) {
                Vector3f target_meas;
                copter.precland.get_target_position_measurement_NED_m(target_meas);
                float meas_alt_m = target_meas.z;  // positive = below drone
                float rf_alt_m = copter.rangefinder_state.alt_m;
                
                // Check for gross mismatch (factor of 5 difference)
                bool alt_mismatch = false;
                if (copter.rangefinder_state.alt_healthy && rf_alt_m > 1.0f) {
                    if (meas_alt_m < rf_alt_m * 0.2f || meas_alt_m > rf_alt_m * 5.0f) {
                        alt_mismatch = true;
                    }
                }
                
                if (alt_mismatch) {
                    // Target position is unreliable - don't use it
                    have_valid_target_pos = false;
                    static uint32_t last_sanity_warn_ms = 0;
                    if ((AP_HAL::millis() - last_sanity_warn_ms) > 5000) {
                        last_sanity_warn_ms = AP_HAL::millis();
                        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "PrecLand: Alt mismatch! Meas=%.1fm RF=%.1fm", 
                                      (double)meas_alt_m, (double)rf_alt_m);
                    }
                }
            }
        }
        
        if (!have_valid_target_pos) {
            // During alignment OR no valid target: Position control strategy
            static Vector2p fine_align_ref_pos;
            static bool fine_align_ref_valid = false;
            
            bool in_fine_phase = (yaw_state == AC_PrecLand::YawAlignState::FINE_ALIGNING ||
                                  yaw_state == AC_PrecLand::YawAlignState::FINE_HOLDING ||
                                  yaw_state == AC_PrecLand::YawAlignState::FINAL_DESCENT);
            
            if (in_fine_phase) {
                // WIND-ROBUST APPROACH: Try to use target position with sanity limits
                Vector2p target_pos_temp;
                bool have_target = copter.precland.get_target_position_m(target_pos_temp);
                
                if (!fine_align_ref_valid) {
                    // Save reference position at entry
                    fine_align_ref_pos = pos_control->get_pos_estimate_NED_m().xy();
                    fine_align_ref_valid = true;
                }
                
                if (have_target) {
                    // Check if target correction is reasonable
                    // PLND_FINE_CORR limits max correction to prevent oscillation while allowing wind compensation
                    const float max_correction_m = copter.precland.get_fine_correction_max_m();
                    
                    if (is_zero(max_correction_m)) {
                        // No limit - use full target position
                        target_pos_ne_m = target_pos_temp;
                    } else {
                        Vector2f correction = (target_pos_temp - fine_align_ref_pos).tofloat();
                        float correction_dist = correction.length();
                        
                        if (correction_dist <= max_correction_m || correction_dist < 0.001f) {
                            // Small correction or negligible - use target position (wind compensation)
                            target_pos_ne_m = target_pos_temp;
                        } else {
                            // Large correction - limit it (probably noise/error)
                            // SAFETY: Only normalize if length is significant to avoid NaN
                            correction *= (max_correction_m / correction_dist);
                            target_pos_ne_m = fine_align_ref_pos + correction.topostype();
                        }
                    }
                } else {
                    // No target - hold reference position
                    target_pos_ne_m = fine_align_ref_pos;
                }
            } else {
                // For other states (SEARCHING, COARSE_ALIGNING, etc.): use current position
                fine_align_ref_valid = false;
                target_pos_ne_m = pos_control->get_pos_estimate_NED_m().xy();
            }
            target_vel_ne_ms.zero();  // Zero velocity to hold position
        } else {
            // Alignment complete and target acquired - use actual target position
            copter.precland.get_target_velocity_ms(pos_control->get_vel_estimate_NED_ms().xy(), target_vel_ne_ms);
        }

        Vector2f accel_zero;
        pos_control->input_pos_vel_accel_NE_m(target_pos_ne_m, target_vel_ne_ms, accel_zero);

        // =============================================================================
        // Yaw control with XY-Gate: only activate when XY tolerance is met
        // =============================================================================
        float target_yaw_rad;
        bool yaw_align_enabled = copter.precland.yaw_align_enabled();
        bool xy_gate_enabled = copter.precland.yaw_xy_gate_enabled();
        
        if (yaw_align_enabled) {
            // Check XY error if gate is enabled
            bool xy_within_tolerance = true;
            if (xy_gate_enabled) {
                // target_pos_ne_m wurde bereits oben deklariert, verwende es hier
                const Vector2p current_pos_ne_m = pos_control->get_pos_estimate_NED_m().xy();
                float target_error_m = (target_pos_ne_m - current_pos_ne_m).tofloat().length();
                const float max_horiz_pos_error_m = copter.precland.get_max_xy_error_before_descending_m();
                xy_within_tolerance = (target_error_m <= max_horiz_pos_error_m || is_zero(max_horiz_pos_error_m));
            }
            
            // Only activate yaw alignment if XY-Gate is satisfied (or disabled)
            if (xy_within_tolerance && copter.precland.get_target_yaw_rad(target_yaw_rad)) {
                float desired_yaw = get_cached_desired_yaw_rad();
                uint8_t state = get_cached_yaw_state();
                
                // FIX: Check if target is still visible/acquired before enabling yaw control
                bool target_ok = copter.precland.target_visible() || copter.precland.target_acquired();
                
                // Only control yaw when in an active alignment state AND target is OK
                bool yaw_control_active = target_ok && 
                    (state != static_cast<uint8_t>(AC_PrecLand::YawAlignState::DISABLED) && 
                     state != static_cast<uint8_t>(AC_PrecLand::YawAlignState::SEARCHING));
                
                if (yaw_control_active) {
                    auto_yaw.set_precland_target_yaw_rad(desired_yaw);
                    if (auto_yaw.mode() != AutoYaw::Mode::PRECLAND_TARGET) {
                        auto_yaw.set_mode(AutoYaw::Mode::PRECLAND_TARGET);
                    }
                } else {
                    // SEARCHING state or target lost - hold yaw immediately
                    // FIX: This prevents feedback loops when target is lost during alignment
                    if (auto_yaw.mode() == AutoYaw::Mode::PRECLAND_TARGET) {
                        auto_yaw.invalidate_precland_target_yaw();
                        auto_yaw.set_mode(AutoYaw::Mode::HOLD);
                    }
                }
            } else {
                // XY-Gate not satisfied or no valid target yaw - hold yaw
                if (auto_yaw.mode() == AutoYaw::Mode::PRECLAND_TARGET) {
                    auto_yaw.invalidate_precland_target_yaw();
                    auto_yaw.set_mode(AutoYaw::Mode::HOLD);
                }
            }
        } else {
            // Yaw-Alignment deaktiviert
            if (auto_yaw.mode() == AutoYaw::Mode::PRECLAND_TARGET) {
                auto_yaw.invalidate_precland_target_yaw();
                auto_yaw.set_mode(AutoYaw::Mode::HOLD);
            }
        }
    } else {
        // Precision landing not active
        if (auto_yaw.mode() == AutoYaw::Mode::PRECLAND_TARGET) {
            auto_yaw.invalidate_precland_target_yaw();
            auto_yaw.set_mode(AutoYaw::Mode::HOLD);
        }
    }
#endif

    if (!copter.ap.prec_land_active) {
        Vector2f accel;
        pos_control->input_vel_accel_NE_m(vel_correction_ms, accel);
    }

    pos_control->NE_update_controller();
    Vector3f thrust_vector = pos_control->get_thrust_vector();

    attitude_control->input_thrust_vector_heading(thrust_vector, auto_yaw.get_heading());
}

// run normal or precision landing (if enabled)
void Mode::land_run_normal_or_precland(bool pause_descent)
{
#if AC_PRECLAND_ENABLED
    if (pause_descent || !copter.precland.enabled()) {
        land_run_horiz_and_vert_control(pause_descent);
        return;
    }
    
    // =============================================================================
    // KRITISCH: Cache MUSS am Anfang aktualisiert werden!
    // =============================================================================
    update_yaw_align_cache();  // ← NEU: Expliziter Aufruf
    
    // Now we can safely read the cache
    bool yaw_pause = false;
    if (copter.precland.yaw_align_enabled()) {
        // Now it's safe to read the cache directly
        // (it was just updated)
        if (!_cached_yaw_allow_descent) {
            yaw_pause = true;
        }
    }
    
    if (yaw_pause) {
        land_run_horiz_and_vert_control(true);
    } else {
        precland_run();
    }
#else
    land_run_horiz_and_vert_control(pause_descent);
#endif
}

#if AC_PRECLAND_ENABLED
// Go towards a position commanded by prec land state machine in order to retry landing
// The passed in location is expected to be NED and in m
void Mode::precland_retry_position(const Vector3p &retry_pos_ned_m)
{
    if (rc().has_valid_input()) {
        if ((g.throttle_behavior & THR_BEHAVE_HIGH_THROTTLE_CANCELS_LAND) != 0 && copter.rc_throttle_control_in_filter.get() > LAND_CANCEL_TRIGGER_THR){
            LOGGER_WRITE_EVENT(LogEvent::LAND_CANCELLED_BY_PILOT);
            // exit land if throttle is high
            if (!set_mode(Mode::Number::LOITER, ModeReason::THROTTLE_LAND_ESCAPE)) {
                set_mode(Mode::Number::ALT_HOLD, ModeReason::THROTTLE_LAND_ESCAPE);
            }
        }

        // allow user to take control during repositioning. Note: copied from land_run_horizontal_control()
        // To-Do: this code exists at several different places in slightly different forms and that should be fixed
        if (g.land_repositioning) {
            float target_roll_rad = 0.0f;
            float target_pitch_rad = 0.0f;
            // convert pilot input to lean angles
            get_pilot_desired_lean_angles_rad(target_roll_rad, target_pitch_rad, loiter_nav->get_angle_max_rad(), attitude_control->get_althold_lean_angle_max_rad());

            // record if pilot has overridden roll or pitch
            if (!is_zero(target_roll_rad) || !is_zero(target_pitch_rad)) {
                if (!copter.ap.land_repo_active) {
                    LOGGER_WRITE_EVENT(LogEvent::LAND_REPO_ACTIVE);
                }
                // this flag will be checked by prec land state machine later and any further landing retires will be cancelled
                copter.ap.land_repo_active = true;
            }
        }
    }

    pos_control->input_pos_NED_m(retry_pos_ned_m, 0.0f, 10.0);

    // run position controllers
    pos_control->NE_update_controller();
    pos_control->D_update_controller();

    // call attitude controller
    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), auto_yaw.get_heading());
}

// Run precland statemachine. This function should be called from any mode that wants to do precision landing.
// This handles everything from prec landing, to prec landing failures, to retries and failsafe measures
void Mode::precland_run()
{
    // if user is taking control, we will not run the statemachine, and simply land (may or may not be on target)
    if (!copter.ap.land_repo_active) {
        // This will get updated later to a retry pos if needed
        Vector3p retry_pos_ned_m;

        switch (copter.precland_statemachine.update(retry_pos_ned_m)) {
        case AC_PrecLand_StateMachine::Status::RETRYING:
            // we want to retry landing by going to another position
            precland_retry_position(retry_pos_ned_m);
            break;

        case AC_PrecLand_StateMachine::Status::FAILSAFE: {
            // we have hit a failsafe. Failsafe can only mean two things, we either want to stop permanently till user takes over or land
            switch (copter.precland_statemachine.get_failsafe_actions()) {
            case AC_PrecLand_StateMachine::FailSafeAction::DESCEND:
                // descend normally, prec land target is definitely not in sight
                land_run_horiz_and_vert_control();
                break;
            case AC_PrecLand_StateMachine::FailSafeAction::HOLD_POS:
                // sending "true" in this argument will stop the descend
                land_run_horiz_and_vert_control(true);
                break;
            }
            break;
        }
        case AC_PrecLand_StateMachine::Status::ERROR:
            // should never happen, is certainly a bug. Report then descend
            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
            FALLTHROUGH;
        case AC_PrecLand_StateMachine::Status::DESCEND:
            // run land controller. This will descend towards the target if prec land target is in sight
            // else it will just descend vertically
            land_run_horiz_and_vert_control();
            break;
        }
    } else {
        // just land, since user has taken over controls, it does not make sense to run any retries or failsafe measures
        land_run_horiz_and_vert_control();
    }
}
#endif

float Mode::throttle_hover() const
{
    return motors->get_throttle_hover();
}

// transform pilot's manual throttle input to make hover throttle mid stick
// used only for manual throttle modes
// thr_mid should be in the range 0 to 1
// returns throttle output 0 to 1
float Mode::get_pilot_desired_throttle() const
{
    int16_t mid_stick = copter.get_throttle_mid();
    // protect against unlikely divide by zero
    if (mid_stick <= 0) {
        mid_stick = 500;
    }

    int16_t throttle_control = channel_throttle->get_control_in();
    // ensure reasonable throttle values
    throttle_control = constrain_int16(throttle_control,0,1000);

    // calculate normalised throttle input
    float throttle_in;
    if (throttle_control < mid_stick) {
        throttle_in = ((float)throttle_control)*0.5f/(float)mid_stick;
    } else {
        throttle_in = 0.5f + ((float)(throttle_control-mid_stick)) * 0.5f / (float)(1000-mid_stick);
    }

    const float thr_mid = throttle_hover();
    const float expo = constrain_float(-(thr_mid-0.5f)/0.375f, -0.5f, 1.0f);
    // calculate the output throttle using the given expo function
    float throttle_out = throttle_in*(1.0f-expo) + expo*throttle_in*throttle_in*throttle_in;
    return throttle_out;
}

float Mode::get_avoidance_adjusted_climbrate_ms(float target_rate_ms)
{
#if AP_AVOIDANCE_ENABLED
    float target_rate_cms = target_rate_ms * 100.0;
    AP::ac_avoid()->adjust_velocity_z(pos_control->D_get_pos_p().kP(), pos_control->D_get_max_accel_mss() * 100.0, target_rate_cms, G_Dt);
    return target_rate_cms * 0.01;
#else
    return target_rate_ms;
#endif
}

// send output to the motors, can be overridden by subclasses
void Mode::output_to_motors()
{
    motors->output();
}

Mode::AltHoldModeState Mode::get_alt_hold_state_D_ms(float target_climb_rate_ms)
{
    // Alt Hold State Machine Determination
    if (!motors->armed()) {
        // the aircraft should moved to a shut down state
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);

        // transition through states as aircraft spools down
        switch (motors->get_spool_state()) {

        case AP_Motors::SpoolState::SHUT_DOWN:
            return AltHoldModeState::MotorStopped;

        case AP_Motors::SpoolState::GROUND_IDLE:
            return AltHoldModeState::Landed_Ground_Idle;

        default:
            return AltHoldModeState::Landed_Pre_Takeoff;
        }

    } else if (takeoff.running() || takeoff.triggered_ms(target_climb_rate_ms)) {
        // the aircraft is currently landed or taking off, asking for a positive climb rate and in THROTTLE_UNLIMITED
        // the aircraft should progress through the take off procedure
        return AltHoldModeState::Takeoff;

    } else if (!copter.ap.auto_armed || copter.ap.land_complete) {
        // the aircraft is armed and landed
        if (target_climb_rate_ms < 0.0f && !copter.ap.using_interlock) {
            // the aircraft should move to a ground idle state
            motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);

        } else {
            // the aircraft should prepare for imminent take off
            motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
        }

        if (motors->get_spool_state() == AP_Motors::SpoolState::GROUND_IDLE) {
            // the aircraft is waiting in ground idle
            return AltHoldModeState::Landed_Ground_Idle;

        } else {
            // the aircraft can leave the ground at any time
            return AltHoldModeState::Landed_Pre_Takeoff;
        }

    } else {
        // the aircraft is in a flying state
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
        return AltHoldModeState::Flying;
    }
}

// transform pilot's yaw input into a desired yaw rate
// returns desired yaw rate in centi-degrees per second
float Mode::get_pilot_desired_yaw_rate_rads() const
{
    if (!rc().has_valid_input()) {
        return 0.0f;
    }

    // Get yaw input
    const float yaw_in = channel_yaw->norm_input_dz();

    // convert pilot input to the desired yaw rate
    return radians(g2.command_model_pilot_y.get_rate()) * input_expo(yaw_in, g2.command_model_pilot_y.get_expo());
}

// pass-through functions to reduce code churn on conversion;
// these are candidates for moving into the Mode base
// class.

// Returns the pilot’s commanded climb rate in m/s.
float Mode::get_pilot_desired_climb_rate_ms() const
{
    return copter.get_pilot_desired_climb_rate_ms();
}

// Returns half the hover throttle.
float Mode::get_non_takeoff_throttle() const
{
    return copter.get_non_takeoff_throttle();
}

// Updates simple/super-simple heading reference based on current yaw and mode.
void Mode::update_simple_mode(void) {
    copter.update_simple_mode();
}

// Requests a mode change with the specified reason; returns true if accepted.
bool Mode::set_mode(Mode::Number mode, ModeReason reason)
{
    return copter.set_mode(mode, reason);
}

// Sets the “land complete” state flag.
void Mode::set_land_complete(bool b)
{
    return copter.set_land_complete(b);
}

// Returns a reference to the GCS interface for Copter.
GCS_Copter &Mode::gcs() const
{
    return copter.gcs();
}

// Returns the pilot’s maximum upward speed in m/s.
float Mode::get_pilot_speed_up_ms() const
{
    return g.pilot_speed_up_cms * 0.01;
}

// Returns the pilot’s maximum downward speed in m/s.
float Mode::get_pilot_speed_dn_ms() const
{
    return copter.get_pilot_speed_dn() * 0.01;
}

// Returns the pilot’s vertical acceleration limit in m/s².
float Mode::get_pilot_accel_D_mss() const
{
    return g.pilot_accel_d_cmss * 0.01;
}

// Return stopping point as a location with above origin alt frame
Location Mode::get_stopping_point() const
{
    Vector3p stopping_point_ned_m;
    copter.pos_control->get_stopping_point_NE_m(stopping_point_ned_m.xy());
    copter.pos_control->get_stopping_point_D_m(stopping_point_ned_m.z);
    return Location::from_ekf_offset_NED_m(stopping_point_ned_m, Location::AltFrame::ABOVE_ORIGIN);
}