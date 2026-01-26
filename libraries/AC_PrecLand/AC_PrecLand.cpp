#include "AC_PrecLand_config.h"

#if AC_PRECLAND_ENABLED

#include "AC_PrecLand.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_AHRS/AP_AHRS.h>

#include "AC_PrecLand_Backend.h"
#include "AC_PrecLand_MAVLink.h"
#include "AC_PrecLand_IRLock.h"
#include "AC_PrecLand_SITL_Gazebo.h"
#include "AC_PrecLand_SITL.h"
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>

extern const AP_HAL::HAL& hal;

#if APM_BUILD_TYPE(APM_BUILD_Rover)
 # define AC_PRECLAND_ORIENT_DEFAULT Rotation::ROTATION_NONE
#else
 # define AC_PRECLAND_ORIENT_DEFAULT Rotation::ROTATION_PITCH_270
#endif

static const uint32_t EKF_INIT_TIME_MS = 2000; // EKF initialisation requires this many milliseconds of good sensor data
static const uint32_t EKF_INIT_SENSOR_MIN_UPDATE_MS = 500; // Sensor must update within this many ms during EKF init, else init will fail
static const uint32_t LANDING_TARGET_TIMEOUT_MS = 2000; // Sensor must update within this many ms, else prec landing will be switched off
static const uint32_t LANDING_TARGET_LOST_TIMEOUT_MS = 180000; // Target will be considered as "lost" if the last known location of the target is more than this many ms ago
static const float    LANDING_TARGET_LOST_DIST_THRESH_M  = 30; // If the last known location of the landing target is beyond this many meters, then we will consider it lost

const AP_Param::GroupInfo AC_PrecLand::var_info[] = {
    // @Param: ENABLED
    // @DisplayName: Precision Land enabled/disabled
    // @Description: Precision Land enabled/disabled
    // @Values: 0:Disabled, 1:Enabled
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLED", 0, AC_PrecLand, _enabled, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: TYPE
    // @DisplayName: Precision Land Type
    // @Description: Precision Land Type
    // @Values: 0:None, 1:MAVLink, 2:IRLock, 3:SITL_Gazebo, 4:SITL
    // @User: Advanced
    AP_GROUPINFO("TYPE",    1, AC_PrecLand, _type, 0),

    // @Param: YAW_ALIGN
    // @DisplayName: Sensor yaw alignment
    // @Description: Yaw angle from body x-axis to sensor x-axis.
    // @Range: 0 36000
    // @Increment: 10
    // @User: Advanced
    // @Units: cdeg
    AP_GROUPINFO("YAW_ALIGN",    2, AC_PrecLand, _yaw_align_cd, 0),

    // @Param: LAND_OFS_X
    // @DisplayName: Land offset forward
    // @Description: Desired landing position of the camera forward of the target in vehicle body frame
    // @Range: -20 20
    // @Increment: 1
    // @User: Advanced
    // @Units: cm
    AP_GROUPINFO("LAND_OFS_X",    3, AC_PrecLand, _land_ofs_cm_x, 0),

    // @Param: LAND_OFS_Y
    // @DisplayName: Land offset right
    // @Description: desired landing position of the camera right of the target in vehicle body frame
    // @Range: -20 20
    // @Increment: 1
    // @User: Advanced
    // @Units: cm
    AP_GROUPINFO("LAND_OFS_Y",    4, AC_PrecLand, _land_ofs_cm_y, 0),

    // @Param: EST_TYPE
    // @DisplayName: Precision Land Estimator Type
    // @Description: Specifies the estimation method to be used
    // @Values: 0:RawSensor, 1:KalmanFilter
    // @User: Advanced
    AP_GROUPINFO("EST_TYPE",    5, AC_PrecLand, _estimator_type, 1),

    // @Param: ACC_P_NSE
    // @DisplayName: Kalman Filter Accelerometer Noise
    // @Description: Kalman Filter Accelerometer Noise, higher values weight the input from the camera more, accels less
    // @Range: 0.5 5
    // @User: Advanced
    AP_GROUPINFO("ACC_P_NSE", 6, AC_PrecLand, _accel_noise, 2.5f),

    // @Param: CAM_POS_X
    // @DisplayName: Camera X position offset
    // @Description: X position of the camera in body frame. Positive X is forward of the origin.
    // @Units: m
    // @Range: -5 5
    // @Increment: 0.01
    // @User: Advanced

    // @Param: CAM_POS_Y
    // @DisplayName: Camera Y position offset
    // @Description: Y position of the camera in body frame. Positive Y is to the right of the origin.
    // @Units: m
    // @Range: -5 5
    // @Increment: 0.01
    // @User: Advanced

    // @Param: CAM_POS_Z
    // @DisplayName: Camera Z position offset
    // @Description: Z position of the camera in body frame. Positive Z is down from the origin.
    // @Units: m
    // @Range: -5 5
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("CAM_POS", 7, AC_PrecLand, _cam_offset_m, 0.0f),

    // @Param: BUS
    // @DisplayName: Sensor Bus
    // @Description: Precland sensor bus for I2C sensors.
    // @Values: -1:DefaultBus,0:InternalI2C,1:ExternalI2C
    // @User: Advanced
    AP_GROUPINFO("BUS",    8, AC_PrecLand, _bus, -1),

    // @Param: LAG
    // @DisplayName: Precision Landing sensor lag
    // @Description: Precision Landing sensor lag, to cope with variable landing_target latency
    // @Range: 0.02 0.250
    // @Increment: 1
    // @Units: s
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("LAG", 9, AC_PrecLand, _lag_s, 0.02f), // 20ms is the old default buffer size (8 frames @ 400hz/2.5ms)

    // @Param: XY_DIST_MAX
    // @DisplayName: Precision Landing maximum distance to target before descending
    // @Description: The vehicle will not start descending if the landing target is detected and it is further than this many meters away. Set 0 to always descend.
    // @Range: 0 10
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("XY_DIST_MAX", 10, AC_PrecLand, _xy_max_dist_desc_m, 2.5f),

    // @Param: STRICT
    // @DisplayName: PrecLand strictness
    // @Description: How strictly should the vehicle land on the target if target is lost
    // @Values: 0: Land Vertically (Not strict), 1: Retry Landing(Normal Strictness), 2: Do not land (just Hover) (Very Strict)
    AP_GROUPINFO("STRICT", 11, AC_PrecLand, _strict, 1),

    // @Param: RET_MAX
    // @DisplayName: PrecLand Maximum number of retires for a failed landing
    // @Description: PrecLand Maximum number of retires for a failed landing. Set to zero to disable landing retry.
    // @Range: 0 10
    // @Increment: 1
    AP_GROUPINFO("RET_MAX", 12, AC_PrecLand, _retry_max, 4),

    // @Param: TIMEOUT
    // @DisplayName: PrecLand retry timeout
    // @Description: Time for which vehicle continues descend even if target is lost. After this time period, vehicle will attempt a landing retry depending on PLND_STRICT parameter.
    // @Range: 0 20
    // @Units: s
    AP_GROUPINFO("TIMEOUT", 13, AC_PrecLand, _retry_timeout_s, 4),

    // @Param: RET_BEHAVE
    // @DisplayName: PrecLand retry behaviour
    // @Description: Prec Land will do the action selected by this parameter if a retry to a landing is needed
    // @Values: 0: Go to the last location where landing target was detected, 1: Go towards the approximate location of the detected landing target
    AP_GROUPINFO("RET_BEHAVE", 14, AC_PrecLand, _retry_behave, 0),

    // @Param: ALT_MIN
    // @DisplayName: PrecLand minimum alt for retry
    // @Description: Vehicle will continue landing vertically even if target is lost below this height. This needs a rangefinder to work. Set to zero to disable this.
    // @Range: 0 5
    // @Units: m
    AP_GROUPINFO("ALT_MIN", 15, AC_PrecLand, _sensor_min_alt_m, 0.75),

    // @Param: ALT_MAX
    // @DisplayName: PrecLand maximum alt for retry
    // @Description: Vehicle will continue landing vertically until this height if target is not found. Below this height if landing target is not found, landing retry/failsafe might be attempted. This needs a rangefinder to work. Set to zero to disable this.
    // @Range: 0 50
    // @Units: m
    AP_GROUPINFO("ALT_MAX", 16, AC_PrecLand, _sensor_max_alt_m, 8),

    // @Param: OPTIONS
    // @DisplayName: Precision Landing Extra Options
    // @Description: Precision Landing Extra Options
    // @Bitmask: 0: Moving Landing Target, 1: Allow Precision Landing after manual reposition, 2: Maintain high speed in final descent
    // @User: Advanced
    AP_GROUPINFO("OPTIONS", 17, AC_PrecLand, _options, 0),

    // @Param: YAW_TGT
    // @DisplayName: Yaw Alignment Target Angle
    // @Description: Desired yaw offset from landing target orientation in centidegrees. 0 means drone nose points same direction as tag's X-axis. Set to -1 to disable yaw alignment feature entirely.
    // @Values: -1:Disabled
    // @Range: -18000 18000
    // @Units: cdeg
    // @User: Advanced
    AP_GROUPINFO("YAW_TGT", 19, AC_PrecLand, _yaw_align_target_cd, 0),
    
    // @Param: YAW_COARSE
    // @DisplayName: Coarse Yaw Alignment Tolerance  
    // @Description: Yaw error tolerance for coarse alignment phase in centidegrees. Drone will not descend until yaw is within this tolerance.
    // @Range: 500 9000
    // @Units: cdeg
    // @User: Advanced
    AP_GROUPINFO("YAW_COARSE", 20, AC_PrecLand, _yaw_coarse_tol_cd, 2000),
    
    // @Param: YAW_FINE
    // @DisplayName: Fine Yaw Alignment Tolerance
    // @Description: Yaw error tolerance for fine alignment phase in centidegrees before final descent.
    // @Range: 100 2000
    // @Units: cdeg
    // @User: Advanced
    AP_GROUPINFO("YAW_FINE", 21, AC_PrecLand, _yaw_fine_tol_cd, 500),
    
    // @Param: YAW_TIME
    // @DisplayName: Yaw Alignment Hold Time
    // @Description: Time in seconds that yaw must be held within tolerance before proceeding.
    // @Range: 1 10
    // @Units: s
    // @User: Advanced
    AP_GROUPINFO("YAW_TIME", 22, AC_PrecLand, _yaw_hold_time_s, 5.0f),
    
    // @Param: YAW_FALT
    // @DisplayName: Fine Yaw Alignment Altitude
    // @Description: Altitude above ground (via rangefinder) at which fine yaw alignment begins. Set 0 to disable.
    // @Range: 0 10
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("YAW_FALT", 23, AC_PrecLand, _yaw_fine_alt_m, 2.0f),
    
    // @Param: YAW_RATE
    // @DisplayName: Maximum Yaw Rate for Alignment
    // @Description: Maximum yaw rate in deg/s during precision landing yaw alignment. Lower values reduce risk of losing target during rotation.
    // @Range: 10 90
    // @Units: deg/s
    // @User: Advanced
    AP_GROUPINFO("YAW_RATE", 24, AC_PrecLand, _yaw_rate_max_dps, 20),
    
    // @Param: YAW_STABLE
    // @DisplayName: Yaw Stable Time
    // @Description: Time in seconds that yaw must be stable within tolerance before descent continues after alignment.
    // @Range: 0 10
    // @Units: s
    // @User: Advanced
    AP_GROUPINFO("YAW_STABLE", 25, AC_PrecLand, _yaw_stable_time_s, 2.0f),
    
    // @Param: ACC_ERR
    // @DisplayName: Acceptable XY Error
    // @Description: Acceptable horizontal position error in meters for slow descent during precision landing.
    // @Range: 0.05 1.0
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("ACC_ERR", 26, AC_PrecLand, _acceptable_error_m, 0.15f),
    
    // @Param: MIN_DSPD
    // @DisplayName: Minimum Descent Speed
    // @Description: Minimum descent speed in m/s during precision landing when close to target.
    // @Range: 0.05 0.5
    // @Units: m/s
    // @User: Advanced
    AP_GROUPINFO("MIN_DSPD", 27, AC_PrecLand, _min_descent_speed_ms, 0.1f),
    
    // @Param: YAW_XY_GATE
    // @DisplayName: Yaw Alignment XY Gate
    // @Description: Enable gate: Yaw alignment only starts when XY position error is within tolerance. Set to 0 to disable gate (yaw aligns immediately when target detected).
    // @Values: 0:Disabled, 1:Enabled
    // @User: Advanced
    AP_GROUPINFO("YAW_XY_GATE", 28, AC_PrecLand, _yaw_xy_gate, 1),
    
    // @Param: YAW_MAXALT
    // @DisplayName: Yaw Alignment Maximum Altitude
    // @Description: Maximum altitude (m) at which yaw alignment starts. If drone is higher, it will descend first before starting yaw alignment. This prevents losing the target during rotation at high altitude. Set to 0 to disable (start alignment immediately).
    // @Range: 0 20
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("YAW_MAXALT", 29, AC_PrecLand, _yaw_max_alt_m, 5.0f),

    // @Param: YAW_FILT
    // @DisplayName: Yaw Filter Alpha
    // @Description: Low-pass filter coefficient for target yaw smoothing. Higher values mean more filtering (slower response, less oscillation). 0 = no filtering, 1 = maximum filtering. Typical range 0.3-0.6.
    // @Range: 0 0.9
    // @User: Advanced
    AP_GROUPINFO("YAW_FILT", 30, AC_PrecLand, _yaw_filter_alpha, 0.5f),

    // @Param: FINE_CORR
    // @DisplayName: Fine Alignment Maximum Correction
    // @Description: Maximum horizontal correction allowed during fine alignment and final descent phases. Limits position corrections to prevent oscillation while still allowing wind compensation. Set to 0 to disable correction limiting (use full target position).
    // @Range: 0 0.5
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("FINE_CORR", 31, AC_PrecLand, _fine_correction_max_m, 0.15f),

    // @Param{Rover,Copter}: ORIENT
    // @DisplayName: Camera Orientation
    // @Description: Orientation of camera/sensor on body
    // @Values: 0:Forward, 4:Back, 25:Down
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO_FRAME("ORIENT", 18, AC_PrecLand, _orient, AC_PRECLAND_ORIENT_DEFAULT, AP_PARAM_FRAME_ROVER | AP_PARAM_FRAME_COPTER | AP_PARAM_FRAME_TRICOPTER | AP_PARAM_FRAME_HELI), 

    AP_GROUPEND
};

// Default constructor.
AC_PrecLand::AC_PrecLand()
{
    if (_singleton != nullptr) {
        AP_HAL::panic("AC_PrecLand must be singleton");
    }
    _singleton = this;

    // set parameters to defaults
    AP_Param::setup_object_defaults(this, var_info);
}

// perform any required initialisation of landing controllers
// update_rate_hz should be the rate at which the update method will be called in hz
void AC_PrecLand::init(uint16_t update_rate_hz)
{
    // exit immediately if init has already been run
    if (_backend != nullptr) {
        return;
    }

    // init as target TARGET_NEVER_SEEN, we will update this later
    _current_target_state = TargetState::TARGET_NEVER_SEEN;

    // initialize target yaw variables
    _target_yaw_rad = 0.0f;
    _target_yaw_valid = false;
    _target_yaw_timestamp_ms = 0;

    // Initialize yaw alignment state machine
    _yaw_align_state = YawAlignState::DISABLED;
    _yaw_align_start_ms = 0;
    _yaw_hold_start_ms = 0;
    _yaw_stable_start_ms = 0;
    _yaw_in_tolerance = false;
    _coarse_align_complete = false;
    _last_desired_yaw_rad = 0.0f;

    // default health to false
    _backend = nullptr;
    _backend_state.healthy = false;

    // create inertial history buffer
    // constrain lag parameter to be within bounds
    _lag_s.set(constrain_float(_lag_s, 0.02f, 0.25f));  // must match LAG parameter range at line 124

    // calculate inertial buffer size from lag and minimum of main loop rate and update_rate_hz argument
    const uint16_t inertial_buffer_size = MAX((uint16_t)roundf(_lag_s * update_rate_hz), 1);

    // instantiate ring buffer to hold inertial history, return on failure so no backends are created
    _inertial_history = NEW_NOTHROW ObjectArray<inertial_data_frame_s>(inertial_buffer_size);
    if (_inertial_history == nullptr) {
        return;
    }

    // instantiate backend based on type parameter
    switch ((Type)(_type.get())) {
        // no type defined
        case Type::NONE:
        default:
            return;
        // companion computer
#if AC_PRECLAND_MAVLINK_ENABLED
        case Type::MAVLINK:
            _backend = NEW_NOTHROW AC_PrecLand_MAVLink(*this, _backend_state);
            break;
        // IR Lock
#endif
#if AC_PRECLAND_IRLOCK_ENABLED
        case Type::IRLOCK:
            _backend = NEW_NOTHROW AC_PrecLand_IRLock(*this, _backend_state);
            break;
#endif
#if AC_PRECLAND_SITL_GAZEBO_ENABLED
        case Type::SITL_GAZEBO:
            _backend = NEW_NOTHROW AC_PrecLand_SITL_Gazebo(*this, _backend_state);
            break;
#endif
#if AC_PRECLAND_SITL_ENABLED
        case Type::SITL:
            _backend = NEW_NOTHROW AC_PrecLand_SITL(*this, _backend_state);
            break;
#endif
    }

    // init backend
    if (_backend != nullptr) {
        _backend->init();
    }

    _approach_vector_body.x = 1;
    _approach_vector_body.rotate(_orient);
}

// update - give chance to driver to get updates from sensor
void AC_PrecLand::update(float rangefinder_alt_cm, bool rangefinder_alt_valid)
{
    // exit immediately if not enabled
    if (_backend == nullptr || _inertial_history == nullptr) {
        return;
    }

    // append current velocity and attitude correction into history buffer
    struct inertial_data_frame_s inertial_data_newest;
    const auto &_ahrs = AP::ahrs();
    _ahrs.getCorrectedDeltaVelocityNED(inertial_data_newest.correctedVehicleDeltaVelocityNED, inertial_data_newest.dt);
    inertial_data_newest.Tbn = _ahrs.get_rotation_body_to_ned();
    Vector3f curr_vel;
    nav_filter_status status;
    if (!_ahrs.get_velocity_NED(curr_vel) || !_ahrs.get_filter_status(status)) {
        inertial_data_newest.inertialNavVelocityValid = false;
    } else {
        inertial_data_newest.inertialNavVelocityValid = status.flags.horiz_vel;
    }
    curr_vel.z = -curr_vel.z;  // NED to NEU
    inertial_data_newest.inertialNavVelocity = curr_vel;

    inertial_data_newest.time_usec = AP_HAL::micros64();
    _inertial_history->push_force(inertial_data_newest);

    const float rangefinder_alt_m = rangefinder_alt_cm*0.01f;  //cm to meter

    // update estimator of target position
    if (_backend != nullptr && _enabled) {
        _backend->update();
        run_estimator(rangefinder_alt_m, rangefinder_alt_valid);
    }

    // check the status of the landing target location
    check_target_status(rangefinder_alt_m, rangefinder_alt_valid);

#if HAL_LOGGING_ENABLED
    const uint32_t now = AP_HAL::millis();
    if (now - _last_log_ms > 40) {  // 25Hz
        _last_log_ms = now;
        Write_Precland();
    }
#endif
}

// check the status of the target
void AC_PrecLand::check_target_status(float rangefinder_alt_m, bool rangefinder_alt_valid)
{
    if (target_acquired()) {
        // target in sight
        _current_target_state = TargetState::TARGET_FOUND;
        // early return because we already know the status
        return;
    }

    // target not in sight
    if (_current_target_state == TargetState::TARGET_FOUND ||
               _current_target_state == TargetState::TARGET_RECENTLY_LOST) {
        // we had target in sight, but not any more, i.e we have lost the target
        _current_target_state = TargetState::TARGET_RECENTLY_LOST;
    } else {
        // we never had the target in sight
        _current_target_state = TargetState::TARGET_NEVER_SEEN;
    }

    // We definitely do not have the target in sight
    // check if the precision landing sensor is supposed to be in range
    // this needs a valid rangefinder to work
    if (!check_if_sensor_in_range(rangefinder_alt_m, rangefinder_alt_valid)) {
        // Target is not in range (vehicle is either too high or too low). Vehicle will not be attempting any sort of landing retries during this period
        _current_target_state = TargetState::TARGET_OUT_OF_RANGE;
        return;
    }

    if (_current_target_state == TargetState::TARGET_RECENTLY_LOST) {
        // check if it's nearby/found recently, else the status will be demoted to "TARGET_LOST"
        Vector2p curr_pos_ne_m;
        if (AP::ahrs().get_relative_position_NE_origin(curr_pos_ne_m)) {
            const float dist_to_last_target_loc_xy = (curr_pos_ne_m - _last_target_pos_rel_origin_ned_m.xy()).length();
            const float dist_to_last_loc_ne_m = (curr_pos_ne_m - _last_vehicle_pos_ned_m.xy()).length();
            if ((AP_HAL::millis() - _last_valid_target_ms) > LANDING_TARGET_LOST_TIMEOUT_MS) {
                // the target has not been seen for a long time
                // might as well consider it as "never seen"
                _current_target_state = TargetState::TARGET_NEVER_SEEN;
                return;
            }

            if ((dist_to_last_target_loc_xy > LANDING_TARGET_LOST_DIST_THRESH_M) || (dist_to_last_loc_ne_m > LANDING_TARGET_LOST_DIST_THRESH_M)) {
                // the last known location of target is too far away
                _current_target_state = TargetState::TARGET_NEVER_SEEN;
                return;
            }
        }
    }
}

// Check if the landing target is supposed to be in sight based on the height of the vehicle from the ground
// This needs a valid rangefinder to work, if the min/max parameters are non zero
bool AC_PrecLand::check_if_sensor_in_range(float rangefinder_alt_m, bool rangefinder_alt_valid) const
{
    if (is_zero(_sensor_max_alt_m) && is_zero(_sensor_min_alt_m)) {
        // no sensor limits have been specified, assume sensor is always in range
        return true;
    }

    if (!rangefinder_alt_valid) {
        // rangefinder isn't healthy. We might be at a very high altitude
        return false;
    }

    if (rangefinder_alt_m > _sensor_max_alt_m && !is_zero(_sensor_max_alt_m)) {
        // this prevents triggering a retry when we are too far away from the target
        return false;
    }

    if (rangefinder_alt_m < _sensor_min_alt_m && !is_zero(_sensor_min_alt_m)) {
        // this prevents triggering a retry when we are very close to the target
        return false;
    }

    // target should be in range
    return true;
}

// returns true when the landing target has been detected
bool AC_PrecLand::target_acquired()
{
    if ((AP_HAL::millis()-_last_update_ms) > LANDING_TARGET_TIMEOUT_MS) {
        if (_target_acquired) {
            // just lost the landing target, inform the user. This message will only be sent once every time target is lost
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "PrecLand: Target Lost");
        }
        // not had a sensor update since a long time
        // probably lost the target
        _estimator_initialized = false;
        _target_acquired = false;
    }
    return _target_acquired;
}

// returns true immediately when a backend measurement is present (without waiting for EKF initialization)
bool AC_PrecLand::target_visible() const
{
    return (AP_HAL::millis() - _last_backend_los_meas_ms) < LANDING_TARGET_TIMEOUT_MS;
}

// returns target position relative to the EKF origin
bool AC_PrecLand::get_target_position_m(Vector2p& ret)
{
    if (!target_acquired()) {
        return false;
    }
    Vector2p curr_pos_ne_m;
    if (!AP::ahrs().get_relative_position_NE_origin(curr_pos_ne_m)) {
        return false;
    }
    ret.x = (_target_pos_rel_out_ne_m.x + curr_pos_ne_m.x);
    ret.y = (_target_pos_rel_out_ne_m.y + curr_pos_ne_m.y);
    return true;
}

// returns target relative position as 3D vector
void AC_PrecLand::get_target_position_measurement_NED_m(Vector3f& ret)
{
    ret = _target_pos_rel_meas_ned_m;
    return;
}

// returns target position relative to vehicle
bool AC_PrecLand::get_target_position_relative_NE_m(Vector2f& ret)
{
    if (!target_acquired()) {
        return false;
    }
    ret = _target_pos_rel_out_ne_m;
    return true;
}

// returns target velocity relative to vehicle
bool AC_PrecLand::get_target_velocity_relative_NE_ms(Vector2f& ret)
{
    if (!target_acquired()) {
        return false;
    }
    ret = _target_vel_rel_out_ne_ms;
    return true;
}

// get the absolute velocity of the vehicle
void AC_PrecLand::get_target_velocity_ms(const Vector2f& vehicle_velocity_ne_ms, Vector2f& target_vel_ne_ms)
{
    if (!(_options & PLND_OPTION_MOVING_TARGET)) {
        // the target should not be moving
        target_vel_ne_ms.zero();
        return;
    }
    if ((EstimatorType)_estimator_type.get() == EstimatorType::RAW_SENSOR) {
        // We do not predict the velocity of the target in this case
        // assume velocity to be zero
        target_vel_ne_ms.zero();
        return;
    }
    Vector2f target_vel_rel_ne_ms;
    if (!get_target_velocity_relative_NE_ms(target_vel_rel_ne_ms)) {
        // Don't know where the target is
        // assume velocity to be zero
        target_vel_ne_ms.zero();
        return;
    }
    // return the absolute velocity
    target_vel_ne_ms  = target_vel_rel_ne_ms + vehicle_velocity_ne_ms;
}

// handle_msg - Process a LANDING_TARGET mavlink message
// handle_msg - Process a LANDING_TARGET mavlink message
void AC_PrecLand::handle_msg(const mavlink_landing_target_t &packet, uint32_t timestamp_ms)
{
    // run backend update
    if (_backend != nullptr) {
        _backend->handle_msg(packet, timestamp_ms);
    }

    // ========================================================================
    // Process quaternion for target yaw orientation
    // ========================================================================
    // The LANDING_TARGET message contains a quaternion q[4] representing
    // the orientation of the landing target. For our use case (AprilTag),
    // we extract the yaw angle to align the vehicle during landing.
    //
    // Quaternion format: q[0]=w, q[1]=x, q[2]=y, q[3]=z
    // For a pure yaw rotation (from OpenMV): q = [cos(Î¸/2), 0, 0, sin(Î¸/2)]
    // ========================================================================

    const float q_w = packet.q[0];
    const float q_x = packet.q[1];
    const float q_y = packet.q[2];
    const float q_z = packet.q[3];

    // Calculate quaternion norm squared for validation
    const float q_norm_sq = q_w*q_w + q_x*q_x + q_y*q_y + q_z*q_z;

    // Quaternion is valid if:
    // 1. Norm² is approximately 1.0 (between 0.9 and 1.1)
    // 2. This also ensures quaternion is not all zeros (default/unset)
    if (q_norm_sq > 0.9f && q_norm_sq < 1.1f) {
        // Extract yaw from quaternion using ZYX Euler convention
        // Formula: yaw = atan2(2*(w*z + x*y), 1 - 2*(y² + z²))
        //
        // For a pure Z-rotation this simplifies to: yaw = 2*atan2(z, w)
        // But we use the general formula for robustness
        float yaw_body_rad = atan2f(2.0f * (q_w * q_z + q_x * q_y),
                                    1.0f - 2.0f * (q_y * q_y + q_z * q_z));

        // =================================================================
        // CRITICAL FIX: Convert yaw from Body frame to NED frame!
        // The quaternion from the camera gives us the tag's yaw RELATIVE to the camera/body.
        // We need the ABSOLUTE tag yaw in NED frame.
        // 
        // target_yaw_NED = target_yaw_body + vehicle_yaw
        //
        // This ensures that when the drone rotates, the computed target_yaw_NED
        // stays constant (because the tag on the ground doesn't move).
        // =================================================================
        float vehicle_yaw_rad = AP::ahrs().get_yaw();
        float yaw_ned_rad = wrap_PI(yaw_body_rad + vehicle_yaw_rad);

        // =================================================================
        // FIX: ALWAYS set yaw when valid quaternion is received!
        // The yaw data is needed for alignment to start.
        // Previously, we only set yaw if target_acquired() or alignment active,
        // but that created a chicken-and-egg problem: alignment needs yaw,
        // but yaw was only set if alignment was active.
        // =================================================================
        if (true) {  // Always process valid quaternion
            // Apply sensor yaw alignment if configured
            // _yaw_align_cd is the yaw angle from body x-axis to sensor x-axis in centidegrees
            if (!is_zero(_yaw_align_cd)) {
                yaw_ned_rad += radians(_yaw_align_cd * 0.01f);
                yaw_ned_rad = wrap_PI(yaw_ned_rad);
            }

            // Store the target yaw orientation (now in NED frame)
            set_target_yaw_rad(yaw_ned_rad, timestamp_ms);
            
            // Yaw conversion debug removed - enable via LOGGING if needed
        }
    }
}

//
// Private methods
//

// run target position estimator
void AC_PrecLand::run_estimator(float rangefinder_alt_m, bool rangefinder_alt_valid)
{
    _inertial_data_delayed = (*_inertial_history)[0];

    switch ((EstimatorType)_estimator_type.get()) {
        case EstimatorType::RAW_SENSOR: {
            // Return if there's any invalid velocity data
            for (uint8_t i=0; i<_inertial_history->available(); i++) {
                const struct inertial_data_frame_s *inertial_data = (*_inertial_history)[i];
                if (!inertial_data->inertialNavVelocityValid) {
                    _target_acquired = false;
                    return;
                }
            }

            // Predict
            if (target_acquired()) {
                _target_pos_rel_est_ne_m.x -= _inertial_data_delayed->inertialNavVelocity.x * _inertial_data_delayed->dt;
                _target_pos_rel_est_ne_m.y -= _inertial_data_delayed->inertialNavVelocity.y * _inertial_data_delayed->dt;
                _target_vel_rel_est_ne_ms.x = -_inertial_data_delayed->inertialNavVelocity.x;
                _target_vel_rel_est_ne_ms.y = -_inertial_data_delayed->inertialNavVelocity.y;
            }

            // Update if a new Line-Of-Sight measurement is available
            if (construct_pos_meas_using_rangefinder(rangefinder_alt_m, rangefinder_alt_valid)) {
                if (!_estimator_initialized) {
                    // Note: "Init Complete" message is sent elsewhere
                    _estimator_initialized = true;
                }
                _target_pos_rel_est_ne_m.x = _target_pos_rel_meas_ned_m.x;
                _target_pos_rel_est_ne_m.y = _target_pos_rel_meas_ned_m.y;
                _target_vel_rel_est_ne_ms.x = -_inertial_data_delayed->inertialNavVelocity.x;
                _target_vel_rel_est_ne_ms.y = -_inertial_data_delayed->inertialNavVelocity.y;

                _last_update_ms = AP_HAL::millis();
                _target_acquired = true;
            }

            // Output prediction
            if (target_acquired()) {
                run_output_prediction();
            }
            break;
        }
        case EstimatorType::KALMAN_FILTER: {
            // Predict
            if (target_acquired() || _estimator_initialized) {
                const float& dt = _inertial_data_delayed->dt;
                const Vector3f& vehicleDelVel = _inertial_data_delayed->correctedVehicleDeltaVelocityNED;

                _ekf_x.predict(dt, -vehicleDelVel.x, _accel_noise*dt);
                _ekf_y.predict(dt, -vehicleDelVel.y, _accel_noise*dt);
            }

            // Update if a new Line-Of-Sight measurement is available
            if (construct_pos_meas_using_rangefinder(rangefinder_alt_m, rangefinder_alt_valid)) {
                float xy_pos_var = sq(_target_pos_rel_meas_ned_m.z*(0.01f + 0.01f*AP::ahrs().get_gyro().length()) + 0.02f);
                if (!_estimator_initialized) {
                    // Note: "Init Complete" message is sent after EKF is ready
                    // start init of EKF. We will let the filter consume the data for a while before it available for consumption
                    // reset filter state
                    if (_inertial_data_delayed->inertialNavVelocityValid) {
                        _ekf_x.init(_target_pos_rel_meas_ned_m.x, xy_pos_var, -_inertial_data_delayed->inertialNavVelocity.x, sq(2.0f));
                        _ekf_y.init(_target_pos_rel_meas_ned_m.y, xy_pos_var, -_inertial_data_delayed->inertialNavVelocity.y, sq(2.0f));
                    } else {
                        _ekf_x.init(_target_pos_rel_meas_ned_m.x, xy_pos_var, 0.0f, sq(10.0f));
                        _ekf_y.init(_target_pos_rel_meas_ned_m.y, xy_pos_var, 0.0f, sq(10.0f));
                    }
                    _last_update_ms = AP_HAL::millis();
                    _estimator_init_ms = AP_HAL::millis();
                    // we have initialized the estimator but will not use the values for sometime so that EKF settles down
                    _estimator_initialized = true;
                } else {
                    float NIS_x = _ekf_x.getPosNIS(_target_pos_rel_meas_ned_m.x, xy_pos_var);
                    float NIS_y = _ekf_y.getPosNIS(_target_pos_rel_meas_ned_m.y, xy_pos_var);
                    if (MAX(NIS_x, NIS_y) < 3.0f || _outlier_reject_count >= 3) {
                        _outlier_reject_count = 0;
                        _ekf_x.fusePos(_target_pos_rel_meas_ned_m.x, xy_pos_var);
                        _ekf_y.fusePos(_target_pos_rel_meas_ned_m.y, xy_pos_var);
                        _last_update_ms = AP_HAL::millis();
                    } else {
                        _outlier_reject_count++;
                    }
                }
            }

            // check EKF was properly initialized when the sensor detected a landing target
            check_ekf_init_timeout();

            // Output prediction
            if (target_acquired()) {
                _target_pos_rel_est_ne_m.x = _ekf_x.getPos();
                _target_pos_rel_est_ne_m.y = _ekf_y.getPos();
                _target_vel_rel_est_ne_ms.x = _ekf_x.getVel();
                _target_vel_rel_est_ne_ms.y = _ekf_y.getVel();

                run_output_prediction();
            }
            break;
        }
    }
}


// check if EKF got the time to initialize when the landing target was first detected
// Expects sensor to update within EKF_INIT_SENSOR_MIN_UPDATE_MS milliseconds till EKF_INIT_TIME_MS milliseconds have passed
// after this period landing target estimates can be used by vehicle
void AC_PrecLand::check_ekf_init_timeout()
{
    if (!target_acquired() && _estimator_initialized) {
        // we have just got the target in sight
        if (AP_HAL::millis()-_last_update_ms > EKF_INIT_SENSOR_MIN_UPDATE_MS) {
            // we have lost the target, not enough readings to initialize the EKF
            _estimator_initialized = false;
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "PrecLand: Init Failed");
        } else if (AP_HAL::millis()-_estimator_init_ms > EKF_INIT_TIME_MS) {
            // the target has been visible for a while, EKF should now have initialized to a good value
            _target_acquired = true;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "PrecLand: Init Complete");
        }
    }
}

// get 3D vector from vehicle to target and frame.  returns true on success, false on failure
bool AC_PrecLand::retrieve_los_meas(Vector3f& target_vec_unit, VectorFrame& frame)
{
    const uint32_t los_meas_time_ms = _backend->los_meas_time_ms();
    if ((los_meas_time_ms != _last_backend_los_meas_ms) && _backend->get_los_meas(target_vec_unit, frame)) {
        _last_backend_los_meas_ms = los_meas_time_ms;
        if (!is_zero(_yaw_align_cd)) {
            // Apply sensor yaw alignment rotation
            target_vec_unit.rotate_xy(cd_to_rad(_yaw_align_cd));
        }

        // rotate vector based on sensor orientation to get correct body frame vector
        if (_orient != ROTATION_PITCH_270) {
            // by default, the vector is constructed downwards in body frame
            // hence, we do not do any rotation if the orientation is downwards
            // if it is some other orientation, we first bring the vector to forward
            // and then we rotate it to desired orientation
            // because the rotations are measured with respect to a vector pointing towards front in body frame
            // for eg, if orientation is back, i.e., ROTATION_YAW_180, 
            // the vector is first brought to front and then rotation by YAW 180 to take it to the back of vehicle
            target_vec_unit.rotate(ROTATION_PITCH_90); // bring vector to front
            target_vec_unit.rotate(_orient);           // rotate it to desired orientation
        }

        return true;
    }
    return false;
}

// If a new measurement was retrieved, sets _target_pos_rel_meas_ned_m and returns true
bool AC_PrecLand::construct_pos_meas_using_rangefinder(float rangefinder_alt_m, bool rangefinder_alt_valid)
{
    Vector3f target_vec_unit;
    VectorFrame target_vec_frame;
    if (retrieve_los_meas(target_vec_unit, target_vec_frame)) {
        _inertial_data_delayed = (*_inertial_history)[0];

        // sanity check vector is pointing in the right direction
        const bool target_vec_valid = target_vec_unit.projected(_approach_vector_body).dot(_approach_vector_body) > 0.0f;

        // calculate 3D vector to target in NED frame
        Vector3f target_vec_unit_ned;
        switch (target_vec_frame) {
        case VectorFrame::BODY_FRD:
            // convert to NED
            target_vec_unit_ned = _inertial_data_delayed->Tbn * target_vec_unit;
            break;
        case VectorFrame::LOCAL_FRD:
            // rotate vector using delayed yaw
            float roll_rad, pitch_rad, yaw_rad;
            _inertial_data_delayed->Tbn.to_euler(&roll_rad, &pitch_rad, &yaw_rad);
            target_vec_unit_ned = target_vec_unit;
            target_vec_unit_ned.rotate_xy(-yaw_rad);
            break;
        }

        const Vector3f approach_vector_NED_m = _inertial_data_delayed->Tbn * _approach_vector_body;
        const bool alt_valid = (rangefinder_alt_valid && rangefinder_alt_m > 0.0f) || (_backend->distance_to_target() > 0.0f);
        if (target_vec_valid && alt_valid) {
            // distance to target and distance to target along approach vector
            float dist_to_target_m, dist_to_target_along_av_m;
            // figure out ned camera orientation w.r.t its offset
            Vector3f cam_pos_ned_m;
            if (!_cam_offset_m.get().is_zero()) {
                // user has specifed offset for camera
                // take its height into account while calculating distance
                cam_pos_ned_m = _inertial_data_delayed->Tbn * _cam_offset_m;
            }
            // FIX: ALWAYS use rangefinder for distance calculation!
            // The backend's distance_to_target() is based on tag size which is often wrong.
            // Rangefinder is much more reliable for altitude/distance measurement.
            // Only use backend distance if rangefinder is not available.
            if (!rangefinder_alt_valid && _backend->distance_to_target() > 0.0f) {
                // sensor has provided distance to landing target (fallback when no rangefinder)
                dist_to_target_m = _backend->distance_to_target();
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "PrecLand: Using backend distance %.1fm (no RF)", (double)dist_to_target_m);
            } else {
                // sensor only knows the horizontal location of the landing target
                // rely on rangefinder for the vertical target
                dist_to_target_along_av_m = MAX(rangefinder_alt_m - cam_pos_ned_m.projected(approach_vector_NED_m).length(), 0.0f);
                
                // SAFETY: Prevent division by zero or very small values
                float projection_length = target_vec_unit_ned.projected(approach_vector_NED_m).length();
                if (projection_length < 0.01f) {
                    // Target vector is nearly perpendicular to approach vector - use rangefinder directly
                    dist_to_target_m = rangefinder_alt_m;
                } else {
                    dist_to_target_m = dist_to_target_along_av_m / projection_length;
                }
                
                // SAFETY: Sanity check - distance should never be negative or extremely large
                if (!isfinite(dist_to_target_m) || dist_to_target_m < 0.0f || dist_to_target_m > 1000.0f) {
                    return false;  // Invalid measurement
                }
            }

            // Compute camera position relative to IMU
            const Vector3f accel_pos_ned_m = _inertial_data_delayed->Tbn * AP::ins().get_imu_pos_offset(AP::ahrs().get_primary_accel_index());
            const Vector3f cam_pos_ned_rel_imu_ned_m = cam_pos_ned_m - accel_pos_ned_m;

            // Compute target position relative to IMU
            _target_pos_rel_meas_ned_m = (target_vec_unit_ned * dist_to_target_m) + cam_pos_ned_rel_imu_ned_m;

            // store the current relative down position so that if we need to retry landing, we know at this height landing target can be found
            const AP_AHRS &_ahrs = AP::ahrs();
            Vector3p pos_NED;
            if (_ahrs.get_relative_position_NED_origin(pos_NED)) {
                _last_target_pos_rel_origin_ned_m.z = pos_NED.z;
                _last_vehicle_pos_ned_m = pos_NED;
            }
            return true;
        }
    }
    return false;
}

// calculate target's position and velocity relative to the vehicle (used as input to position controller)
// results are stored in_target_pos_rel_out_NE, _target_vel_rel_out_ne_ms
void AC_PrecLand::run_output_prediction()
{
    _target_pos_rel_out_ne_m = _target_pos_rel_est_ne_m;
    _target_vel_rel_out_ne_ms = _target_vel_rel_est_ne_ms;

    // Predict forward from delayed time horizon
    for (uint8_t i=1; i<_inertial_history->available(); i++) {
        const struct inertial_data_frame_s *inertial_data = (*_inertial_history)[i];
        _target_vel_rel_out_ne_ms.x -= inertial_data->correctedVehicleDeltaVelocityNED.x;
        _target_vel_rel_out_ne_ms.y -= inertial_data->correctedVehicleDeltaVelocityNED.y;
        _target_pos_rel_out_ne_m.x += _target_vel_rel_out_ne_ms.x * inertial_data->dt;
        _target_pos_rel_out_ne_m.y += _target_vel_rel_out_ne_ms.y * inertial_data->dt;
    }

    const AP_AHRS &_ahrs = AP::ahrs();

    const Matrix3f& Tbn = (*_inertial_history)[_inertial_history->available()-1]->Tbn;
    Vector3f accel_body_offset = AP::ins().get_imu_pos_offset(_ahrs.get_primary_accel_index());

    // Apply position correction for CG offset from IMU
    Vector3f imu_pos_ned = Tbn * accel_body_offset;
    _target_pos_rel_out_ne_m.x += imu_pos_ned.x;
    _target_pos_rel_out_ne_m.y += imu_pos_ned.y;

    // Apply position correction for body-frame horizontal camera offset from CG, so that vehicle lands lens-to-target
    Vector3f cam_pos_horizontal_ned = Tbn * Vector3f{_cam_offset_m.get().x, _cam_offset_m.get().y, 0};
    _target_pos_rel_out_ne_m.x -= cam_pos_horizontal_ned.x;
    _target_pos_rel_out_ne_m.y -= cam_pos_horizontal_ned.y;

    // Apply velocity correction for IMU offset from CG
    Vector3f vel_ned_rel_imu = Tbn * (_ahrs.get_gyro() % (-accel_body_offset));
    _target_vel_rel_out_ne_ms.x -= vel_ned_rel_imu.x;
    _target_vel_rel_out_ne_ms.y -= vel_ned_rel_imu.y;

    // remember vehicle velocity
    UNUSED_RESULT(_ahrs.get_velocity_NED(_last_veh_velocity_NED_ms));

    // Apply land offset
    Vector3f land_ofs_ned_m = _ahrs.get_rotation_body_to_ned() * Vector3f{_land_ofs_cm_x, _land_ofs_cm_y, 0} * 0.01f;
    _target_pos_rel_out_ne_m.x += land_ofs_ned_m.x;
    _target_pos_rel_out_ne_m.y += land_ofs_ned_m.y;

    // store the landing target as a offset from current position. This is used in landing retry
    Vector2p last_target_loc_rel_origin_ne_m;
    get_target_position_m(last_target_loc_rel_origin_ne_m);
    _last_target_pos_rel_origin_ned_m.x = last_target_loc_rel_origin_ne_m.x;
    _last_target_pos_rel_origin_ned_m.y = last_target_loc_rel_origin_ne_m.y;

    // record the last time there was a target output
    _last_valid_target_ms = AP_HAL::millis();
}

/*
  get target location lat/lon. Note that altitude in returned
  location is not reliable
 */
bool AC_PrecLand::get_target_location(Location &loc)
{
    if (!target_acquired()) {
        return false;
    }
    if (!AP::ahrs().get_origin(loc)) {
        return false;
    }
    loc.offset(_last_target_pos_rel_origin_ned_m.x, _last_target_pos_rel_origin_ned_m.y);
    loc.offset_up_m(-_last_target_pos_rel_origin_ned_m.z);
    return true;
}

/*
  get the absolute velocity of the target in m/s.
  return false if we cannot estimate target velocity or if the target is not acquired
*/
bool AC_PrecLand::get_target_velocity(Vector2f& target_vel)
{
    if (!(_options & PLND_OPTION_MOVING_TARGET)) {
        // the target should not be moving
        return false;
    }
    if ((EstimatorType)_estimator_type.get() == EstimatorType::RAW_SENSOR) {
        return false;
    }
    Vector2f target_vel_rel_ne_ms;
    if (!get_target_velocity_relative_NE_ms(target_vel_rel_ne_ms)) {
        return false;
    }
    // return the absolute velocity
    target_vel = (target_vel_rel_ne_ms) + _last_veh_velocity_NED_ms.xy();
    return true;
}

#if HAL_LOGGING_ENABLED
// Write a precision landing entry
void AC_PrecLand::Write_Precland()
{
    // exit immediately if not enabled
    if (!enabled()) {
        return;
    }

    Vector2f target_pos_rel_ne_m;
    Vector2f target_vel_rel_ne_ms;
    Vector3f target_pos_meas_ned_m;
    get_target_position_relative_NE_m(target_pos_rel_ne_m);
    get_target_velocity_relative_NE_ms(target_vel_rel_ne_ms);
    get_target_position_measurement_NED_m(target_pos_meas_ned_m);

    const struct log_Precland pkt {
        LOG_PACKET_HEADER_INIT(LOG_PRECLAND_MSG),
        time_us         : AP_HAL::micros64(),
        healthy         : healthy(),
        target_acquired : target_acquired(),
        pos_x           : target_pos_rel_ne_m.x,
        pos_y           : target_pos_rel_ne_m.y,
        vel_x           : target_vel_rel_ne_ms.x,
        vel_y           : target_vel_rel_ne_ms.y,
        meas_x          : target_pos_meas_ned_m.x,
        meas_y          : target_pos_meas_ned_m.y,
        meas_z          : target_pos_meas_ned_m.z,
        last_meas       : last_backend_los_meas_ms(),
        ekf_outcount    : ekf_outlier_count(),
        estimator       : (uint8_t)_estimator_type
    };
    AP::logger().WriteBlock(&pkt, sizeof(pkt));
}

// =============================================================================
// Yaw Alignment State Machine Implementation
// =============================================================================

void AC_PrecLand::yaw_align_init()
{
    // Check if yaw alignment is enabled via parameter
    // Value of -1 explicitly disables the feature
    if (_yaw_align_target_cd.get() == -1) {
        _yaw_align_state = YawAlignState::DISABLED;
        _coarse_align_complete = false;
        _yaw_in_tolerance = false;
        _yaw_hold_start_ms = 0;
        _yaw_align_start_ms = 0;
        _target_lost_timestamp_ms = 0;
        _last_desired_yaw_rad = 0.0f;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "PrecLand: Yaw align disabled");
        return;
    }
    
    _yaw_align_state = YawAlignState::SEARCHING;
    _yaw_align_start_ms = AP_HAL::millis();
    _yaw_hold_start_ms = 0;
    _yaw_stable_start_ms = 0;
    _yaw_in_tolerance = false;
    _coarse_align_complete = false;
    _target_lost_timestamp_ms = 0;          
    _last_desired_yaw_rad = AP::ahrs().get_yaw_rad();
    
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "PrecLand: Yaw align active, searching");
}

bool AC_PrecLand::get_desired_yaw_for_alignment(float &yaw_rad) const
{
    // First check if we have a valid target yaw from the sensor
    float target_yaw;
    if (!get_target_yaw_rad(target_yaw)) {
        return false;
    }
    
    // Add the user-configured alignment offset
    float offset_rad = radians(_yaw_align_target_cd.get() * 0.01f);
    
    // Calculate desired yaw: target orientation + user offset
    // The result is the yaw the drone should point to be "aligned" with the target
    yaw_rad = wrap_PI(target_yaw + offset_rad);
    
    return true;
}

float AC_PrecLand::get_yaw_alignment_error_rad() const
{
    float desired_yaw;
    if (!get_desired_yaw_for_alignment(desired_yaw)) {
        return 0.0f;
    }
    
    // Get current vehicle yaw
    const float current_yaw = AP::ahrs().get_yaw_rad();
    
    // Calculate error (positive = need to turn right/clockwise)
    float error = wrap_PI(desired_yaw - current_yaw);
    
    return error;
}

bool AC_PrecLand::yaw_within_tolerance(float tolerance_cd) const
{
    float error_rad = get_yaw_alignment_error_rad();
    float error_cd = degrees(fabsf(error_rad)) * 100.0f;
    return error_cd <= tolerance_cd;
}

AC_PrecLand::YawAlignResult AC_PrecLand::yaw_align_update(float rangefinder_alt_m)
{
    YawAlignResult result;
    result.allow_descent = true;
    result.yaw_aligned = false;
    result.desired_yaw_rad = _last_desired_yaw_rad;
    result.yaw_error_deg = 0.0f;
    result.state = _yaw_align_state;
    
    // SAFETY: If feature was disabled via parameter change, reset state machine
    if (!yaw_align_enabled() && _yaw_align_state != YawAlignState::DISABLED) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "PrecLand: Yaw align disabled via parameter");
        _yaw_align_state = YawAlignState::DISABLED;
        result.state = YawAlignState::DISABLED;
        return result;
    }
    
    // If disabled, always allow descent
    if (_yaw_align_state == YawAlignState::DISABLED) {
        return result;
    }
    
    const uint32_t now_ms = AP_HAL::millis();
    
    // SAFETY: Validate and constrain parameters to sensible ranges
    const float hold_time_ms = MAX(_yaw_hold_time_s.get(), 0.1f) * 1000.0f;  // Min 100ms
    const float coarse_tol_cd = MAX(_yaw_coarse_tol_cd.get(), 100.0f);       // Min 1 degree
    const float fine_tol_cd = MAX(_yaw_fine_tol_cd.get(), 50.0f);            // Min 0.5 degree
    const float fine_alt_m = MAX(_yaw_fine_alt_m.get(), 0.0f);               // Min 0m (disabled)
    const uint32_t ALIGNMENT_TIMEOUT_MS = 30000;
    
    // Get target yaw if available
    float desired_yaw;
    bool have_target_yaw = get_desired_yaw_for_alignment(desired_yaw);
    
    // Periodic debug removed - state changes are logged individually
    
    // Calculate current yaw error
    float yaw_error_rad = get_yaw_alignment_error_rad();
    result.yaw_error_deg = degrees(yaw_error_rad);
    
    // Check tolerances
    bool within_coarse = yaw_within_tolerance(coarse_tol_cd);
    bool within_fine = yaw_within_tolerance(fine_tol_cd);
    
    // State machine
    switch (_yaw_align_state) {
        
    case YawAlignState::DISABLED:
        result.allow_descent = true;
        break;
        
    case YawAlignState::SEARCHING: {
        result.allow_descent = true;
        
        // =================================================================
        // FIX: Check for fresh backend measurement instead of target_acquired()
        // This allows immediate reaction to target detection without waiting
        // for EKF initialization (2 second delay)
        // =================================================================
        bool have_fresh_measurement = target_visible();
        
        // =================================================================
        // NEW: Check altitude limit for yaw alignment
        // If we're too high, continue descending instead of starting alignment
        // This prevents losing the target during rotation at high altitude
        // =================================================================
        bool altitude_ok = true;
        if (!is_zero(_yaw_max_alt_m.get())) {
            // Get current altitude from target measurement (z is positive down)
            Vector3f target_pos_meas;
            get_target_position_measurement_NED_m(target_pos_meas);
            float current_alt_m = target_pos_meas.z;  // positive = below drone
            
            if (current_alt_m > _yaw_max_alt_m.get()) {
                // Too high - continue descending (no message to reduce spam)
                altitude_ok = false;
            }
        }
        
        // =================================================================
        // FIXED LOGIC:
        // 1. If too high -> descent allowed, don't start alignment yet
        // 2. If altitude OK and have measurement + yaw -> start alignment
        // 3. If altitude OK and have measurement but no yaw -> pause and wait
        // =================================================================
        if (!altitude_ok) {
            // Too high - just descend, don't do anything else
            result.allow_descent = true;
            // Message already printed above
        } else if (have_fresh_measurement && have_target_yaw) {
            // Altitude OK, have target and yaw -> START ALIGNMENT!
            _yaw_align_state = YawAlignState::COARSE_ALIGNING;
            _yaw_align_start_ms = now_ms;
            _target_lost_timestamp_ms = 0;
            _yaw_in_tolerance = false;  // Reset tolerance tracker
            _yaw_hold_start_ms = 0;
            _yaw_stable_start_ms = 0;
            
            // Set desired yaw
            result.desired_yaw_rad = desired_yaw;
            _last_desired_yaw_rad = desired_yaw;
            
            // Pause descent during alignment
            result.allow_descent = false;
            
            result.state = YawAlignState::COARSE_ALIGNING;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "PrecLand: Target found, coarse aligning (descent paused)");
        } else if (have_fresh_measurement && !have_target_yaw) {
            // Altitude OK, have target but NO yaw -> wait silently
            result.allow_descent = false;
        }
        // else: no measurement -> allow_descent stays true (default)
        break;
    }
        
    case YawAlignState::COARSE_ALIGNING: {
        result.allow_descent = false;  // Default: NO descent!
        
        if ((now_ms - _yaw_align_start_ms) > ALIGNMENT_TIMEOUT_MS) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "PrecLand: Yaw align timeout");
            _yaw_align_state = YawAlignState::DESCENDING;
            result.allow_descent = true;
            break;
        }
        
        // =================================================================
        // Target loss handling: If target is lost, descend slowly to re-acquire it
        // =================================================================
        // FIX: Check both target_acquired() and target_visible() for consistency
        bool target_still_visible = target_visible();
        if (!target_acquired() && !target_still_visible) {
            // FIX: If alignment is already complete (yaw was in tolerance), don't go back to SEARCHING
            // Instead, transition to DESCENDING to allow descent even if target is lost
            // Check if alignment was successful by checking if hold timer was started
            // This works even if _yaw_in_tolerance was reset due to missing target yaw
            if (_yaw_hold_start_ms > 0) {
                // Alignment was successful (hold timer was started) - transition to DESCENDING instead of SEARCHING
                _yaw_align_state = YawAlignState::DESCENDING;
                _target_lost_timestamp_ms = 0;
                // Keep _yaw_in_tolerance true if it was true, don't reset it
                result.allow_descent = true;
                result.desired_yaw_rad = _last_desired_yaw_rad;
                result.state = YawAlignState::DESCENDING;
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "PrecLand: Target lost after alignment, continuing descent");
                break;
            }
            
            if (_target_lost_timestamp_ms == 0) {
                _target_lost_timestamp_ms = now_ms;
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "PrecLand: Target lost during alignment");
                // FIX: Don't change desired_yaw when target is lost - keep last known value
                // This prevents yaw jumps that cause oscillation
            }
            
            // FIX: Keep desired_yaw at last known value, don't update it
            // This prevents feedback loops when target is temporarily lost
            result.desired_yaw_rad = _last_desired_yaw_rad;
            
            // =================================================================
            // NEW: Allow SLOW descent when target is lost during alignment
            // This helps re-acquire the target by getting closer to it
            // The drone holds yaw (no rotation) and descends slowly
            // =================================================================
            float yaw_rate_dps = fabsf(degrees(AP::ahrs().get_yaw_rate_earth()));
            if (yaw_rate_dps < 5.0f) {
                // Rotation has stopped - allow slow descent to find target again
                result.allow_descent = true;  // Allow descent to get closer to target
                // Only print once
                static uint32_t last_descent_msg_ms = 0;
                if ((now_ms - last_descent_msg_ms) > 3000) {
                    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "PrecLand: Target lost, descending slowly to re-acquire");
                    last_descent_msg_ms = now_ms;
                }
            } else {
                // Still rotating - wait for rotation to stop first
                result.allow_descent = false;
                // Wait silently for rotation to stop
            }
            
        // Use PLND_TIMEOUT parameter for configurable wait time
        const uint32_t target_lost_timeout_ms = MAX(_retry_timeout_s.get(), 2.0f) * 1000;
        if ((now_ms - _target_lost_timestamp_ms) > target_lost_timeout_ms) {
            _yaw_align_state = YawAlignState::SEARCHING;
            _target_lost_timestamp_ms = 0;
            _yaw_in_tolerance = false;
            _yaw_stable_start_ms = 0;
            
            // Only allow descent when rotation rate < 5 deg/s
            if (yaw_rate_dps < 5.0f) {
                result.allow_descent = true;
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "PrecLand: Target lost, back to searching");
            } else {
                result.allow_descent = false;
                // Wait silently for rotation to stop
            }
            } else {
                // FIX: Don't allow descent while waiting for rotation to stop
                result.allow_descent = false;
            }
            // While waiting for target: descent PAUSED, yaw should be set to HOLD
            result.state = YawAlignState::SEARCHING;
            break;
        }
        
        // FIX: If target was lost but is now visible again, reset lost timestamp
        if (!target_acquired() && target_still_visible) {
            // Target visible but EKF not initialized yet - keep waiting
            // Don't reset lost timestamp, but also don't go to SEARCHING
            // Keep in COARSE_ALIGNING state but pause yaw control
            result.desired_yaw_rad = _last_desired_yaw_rad;  // Keep last known yaw
            result.allow_descent = false;
            result.state = _yaw_align_state;
            break;
        }
        
        // Target is visible - reset lost timer
        _target_lost_timestamp_ms = 0;
        
        // Update desired yaw: use measured value if available, otherwise use last known value
        if (have_target_yaw) {
            result.desired_yaw_rad = desired_yaw;
            _last_desired_yaw_rad = desired_yaw;
        } else {
            // FIX: Use last known desired yaw instead of current vehicle yaw
            // This prevents yaw jumps that cause oscillation
            result.desired_yaw_rad = _last_desired_yaw_rad;
        }
        
        // Check if we are within tolerance (only when yaw data is available)
        // FIX: Add hysteresis to prevent oscillation - only reset tolerance if error is significant
        if (have_target_yaw && within_coarse) {
            if (!_yaw_in_tolerance) {
                _yaw_in_tolerance = true;
                _yaw_hold_start_ms = now_ms;
                _yaw_stable_start_ms = now_ms;  // Start hover timer
            }
            
            // Check if hold time has been reached
            if ((now_ms - _yaw_hold_start_ms) >= (uint32_t)hold_time_ms) {
                // Check if yaw has been stable long enough (hover timer)
                const float stable_time_ms = _yaw_stable_time_s.get() * 1000.0f;
                if ((now_ms - _yaw_stable_start_ms) >= (uint32_t)stable_time_ms) {
                    _yaw_align_state = YawAlignState::COARSE_HOLDING;
                    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "PrecLand: Coarse alignment complete!");
                }
            }
        } else {
            // Outside tolerance or no yaw data available
            // FIX: Add hysteresis - only reset if error is significantly outside tolerance
            // This prevents oscillation when yaw is near the tolerance boundary
            if (_yaw_in_tolerance && have_target_yaw) {
                // Check if error is significantly outside tolerance (add 20% margin)
                float error_cd = degrees(fabsf(yaw_error_rad)) * 100.0f;
                float tolerance_with_hysteresis = coarse_tol_cd * 1.2f;
                if (error_cd > tolerance_with_hysteresis) {
                    _yaw_in_tolerance = false;
                    _yaw_hold_start_ms = 0;
                    _yaw_stable_start_ms = 0;
                }
                // If error is between tolerance and tolerance*1.2, keep _yaw_in_tolerance true
                // This creates hysteresis and prevents oscillation
            } else if (!have_target_yaw) {
                // FIX: Don't reset tolerance if alignment was already successful
                // If hold timer was running, keep _yaw_in_tolerance true even if target is lost
                // This allows transition to DESCENDING when target is lost after successful alignment
                if (_yaw_hold_start_ms == 0) {
                    // No yaw data and alignment never started - reset tolerance
                    _yaw_in_tolerance = false;
                    _yaw_stable_start_ms = 0;
                }
                // If _yaw_hold_start_ms > 0, alignment was successful, keep _yaw_in_tolerance true
            }
        }
        
        result.yaw_aligned = have_target_yaw && within_coarse;
        result.state = _yaw_align_state;
        break;
    }
        
    case YawAlignState::COARSE_HOLDING:
        // Brief transition state
        _coarse_align_complete = true;
        _yaw_align_state = YawAlignState::DESCENDING;
        _yaw_in_tolerance = false;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "PrecLand: Descending with yaw control active");
        FALLTHROUGH;
        
    case YawAlignState::DESCENDING: {
        // =================================================================
        // During descent: continue yaw control but allow descent
        // =================================================================
        result.allow_descent = true;
        
        // Target loss handling during descent
        bool curr_visible = target_visible();
        bool curr_acquired = target_acquired();
        static uint32_t descending_target_lost_ms = 0;
        
        if (!curr_acquired && !curr_visible) {
            // Target completely lost - hold position, keep last yaw
            if (descending_target_lost_ms == 0) {
                descending_target_lost_ms = now_ms;
            }
            result.desired_yaw_rad = _last_desired_yaw_rad;
        } else {
            descending_target_lost_ms = 0;
        }
        
        if (have_target_yaw) {
            result.desired_yaw_rad = desired_yaw;
            _last_desired_yaw_rad = desired_yaw;
        }
        
        result.yaw_aligned = within_coarse;
        
        // Check if fine alignment altitude has been reached
        if (fine_alt_m > 0.01f && rangefinder_alt_m > 0.0f && rangefinder_alt_m <= fine_alt_m) {
            _yaw_align_state = YawAlignState::FINE_ALIGNING;
            _yaw_in_tolerance = false;
            _yaw_hold_start_ms = 0;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "PrecLand: Fine align at %.1fm (descent paused)", (double)rangefinder_alt_m);
        }
        
        result.state = _yaw_align_state;
        break;
    }
        
    case YawAlignState::FINE_ALIGNING: {
        // =================================================================
        // Fine aligning: descent paused
        // =================================================================
        result.allow_descent = false;
        
        // SAFETY: Timeout for fine alignment (prevents indefinite hover if sensor fails)
        static uint32_t fine_align_start_ms = 0;
        if (fine_align_start_ms == 0) {
            fine_align_start_ms = now_ms;
        }
        const uint32_t FINE_ALIGN_TIMEOUT_MS = 10000;  // 10 second max
        if ((now_ms - fine_align_start_ms) > FINE_ALIGN_TIMEOUT_MS) {
            _yaw_align_state = YawAlignState::FINAL_DESCENT;
            result.allow_descent = true;
            fine_align_start_ms = 0;  // Reset for next time
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "PrecLand: Fine align timeout, final descent");
            result.state = YawAlignState::FINAL_DESCENT;
            break;
        }
        
        // =================================================================
        // ROBUSTNESS: Target loss handling during fine alignment
        // Wait briefly for target to reappear before giving up
        // =================================================================
        static uint32_t fine_target_lost_ms = 0;
        const uint32_t FINE_TARGET_WAIT_MS = 2000;  // Wait 2 seconds for re-acquisition
        
        if (!target_acquired()) {
            if (fine_target_lost_ms == 0) {
                fine_target_lost_ms = now_ms;
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "PrecLand: Target lost at fine align, waiting %.1fs", 
                              (double)(FINE_TARGET_WAIT_MS / 1000.0f));
            }
            
            // Keep hovering and holding position during wait period
            result.allow_descent = false;
            result.desired_yaw_rad = _last_desired_yaw_rad;  // Hold last yaw
            
            // Check if wait time exceeded
            if ((now_ms - fine_target_lost_ms) > FINE_TARGET_WAIT_MS) {
                _yaw_align_state = YawAlignState::FINAL_DESCENT;
                result.allow_descent = true;
                fine_align_start_ms = 0;  // Reset for next time
                fine_target_lost_ms = 0;  // Reset for next time
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "PrecLand: Target not re-acquired, final descent");
                result.state = YawAlignState::FINAL_DESCENT;
            }
            break;
        } else {
            // Target visible again - reset lost timer
            if (fine_target_lost_ms != 0) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "PrecLand: Target re-acquired at fine align");
                fine_target_lost_ms = 0;
            }
        }
        
        // Update desired yaw: use measured value if available, otherwise use current vehicle yaw
        if (have_target_yaw) {
            result.desired_yaw_rad = desired_yaw;
            _last_desired_yaw_rad = desired_yaw;
        } else {
            result.desired_yaw_rad = AP::ahrs().get_yaw_rad();
            _last_desired_yaw_rad = result.desired_yaw_rad;
        }
        
        // Check if we are within tolerance (only when yaw data is available)
        if (have_target_yaw && within_fine) {
            if (!_yaw_in_tolerance) {
                _yaw_in_tolerance = true;
                _yaw_hold_start_ms = now_ms;
                _yaw_stable_start_ms = now_ms;  // Start hover timer
            }
            
            // Check if hold time has been reached
            if ((now_ms - _yaw_hold_start_ms) >= (uint32_t)hold_time_ms) {
                // Check if yaw has been stable long enough (hover timer)
                const float stable_time_ms = _yaw_stable_time_s.get() * 1000.0f;
                if ((now_ms - _yaw_stable_start_ms) >= (uint32_t)stable_time_ms) {
                    _yaw_align_state = YawAlignState::FINE_HOLDING;
                    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "PrecLand: Fine alignment complete!");
                }
            }
        } else {
            _yaw_in_tolerance = false;
            _yaw_hold_start_ms = 0;
            _yaw_stable_start_ms = 0;
        }
        
        result.yaw_aligned = have_target_yaw && within_fine;
        result.state = _yaw_align_state;
        break;
    }
        
    case YawAlignState::FINE_HOLDING:
        // Brief transition state
        _yaw_align_state = YawAlignState::FINAL_DESCENT;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "PrecLand: Final descent with yaw locked");
        FALLTHROUGH;
        
    case YawAlignState::FINAL_DESCENT:
        // =================================================================
        // Final descent: descent allowed, yaw control continues
        // =================================================================
        result.allow_descent = true;
        
        if (have_target_yaw) {
            result.desired_yaw_rad = desired_yaw;
            _last_desired_yaw_rad = desired_yaw;
        }
        
        result.yaw_aligned = within_fine;
        result.state = _yaw_align_state;
        break;
    }
    
    return result;
}

#endif

// ============================================================================
// Target Yaw Orientation Functions (NUR EINMAL, AUSSERHALB von HAL_LOGGING!)
// ============================================================================

void AC_PrecLand::set_target_yaw_rad(float yaw_rad, uint32_t timestamp_ms)
{
    // SAFETY: Reject invalid input
    if (!isfinite(yaw_rad)) {
        return;  // Don't update with NaN or Inf
    }
    
    float new_yaw = wrap_PI(yaw_rad);
    
    // Apply low-pass filter if configured (PLND_YAW_FILT > 0)
    // Filter uses angular difference to handle wrap-around correctly
    // alpha = 0: no filtering (immediate response)
    // alpha = 0.9: heavy filtering (slow response, very stable)
    const float alpha = constrain_float(_yaw_filter_alpha.get(), 0.0f, 0.9f);
    
    if (_target_yaw_valid && !is_zero(alpha)) {
        // Calculate angular difference (handles wrap-around correctly)
        float yaw_diff = wrap_PI(new_yaw - _target_yaw_rad);
        // Apply filter: filtered = old + (1-alpha) * diff
        // Note: (1-alpha) because alpha=0 means no filtering (full response)
        _target_yaw_rad = wrap_PI(_target_yaw_rad + (1.0f - alpha) * yaw_diff);
    } else {
        // First measurement or no filtering - use raw value
        _target_yaw_rad = new_yaw;
    }
    
    _target_yaw_valid = true;
    _target_yaw_timestamp_ms = timestamp_ms;
}

bool AC_PrecLand::get_target_yaw_rad(float &yaw_rad) const
{
    if (!_target_yaw_valid) {
        return false;
    }
    uint32_t age_ms = AP_HAL::millis() - _target_yaw_timestamp_ms;
    if (age_ms > LANDING_TARGET_TIMEOUT_MS) {
        return false;
    }
    yaw_rad = _target_yaw_rad;
    return true;
}

bool AC_PrecLand::target_yaw_valid() const
{
    if (!_target_yaw_valid) {
        return false;
    }
    return (AP_HAL::millis() - _target_yaw_timestamp_ms) <= LANDING_TARGET_TIMEOUT_MS;
}

// singleton instance
AC_PrecLand *AC_PrecLand::_singleton;

namespace AP {
AC_PrecLand *ac_precland()
{
    return AC_PrecLand::get_singleton();
}
}

#endif // AC_PRECLAND_ENABLED