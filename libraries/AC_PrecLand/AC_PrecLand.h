#pragma once

#include "AC_PrecLand_config.h"

#if AC_PRECLAND_ENABLED

#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_Math/AP_Math.h>
#include <stdint.h>
#include "PosVelEKF.h"
#include <AP_HAL/utility/RingBuffer.h>
#include <AC_PrecLand/AC_PrecLand_StateMachine.h>

// declare backend classes
class AC_PrecLand_Backend;
class AC_PrecLand_MAVLink;
class AC_PrecLand_IRLock;
class AC_PrecLand_SITL_Gazebo;
class AC_PrecLand_SITL;
class Location;

class AC_PrecLand
{
    // declare backends as friends
    friend class AC_PrecLand_Backend;
    friend class AC_PrecLand_MAVLink;
    friend class AC_PrecLand_IRLock;
    friend class AC_PrecLand_SITL_Gazebo;
    friend class AC_PrecLand_SITL;

public:
    AC_PrecLand();

    /* Do not allow copies */
    CLASS_NO_COPY(AC_PrecLand);

    // return singleton
    static AC_PrecLand *get_singleton() {
        return _singleton;
    }

    // perform any required initialisation of landing controllers
    // update_rate_hz should be the rate at which the update method will be called in hz
    void init(uint16_t update_rate_hz);

    // returns true if precision landing is healthy
    bool healthy() const { return _backend_state.healthy; }

    // returns true if precision landing is enabled (used only for logging)
    bool enabled() const { return _enabled.get(); }

    // returns time of last update
    uint32_t last_update_ms() const { return _last_update_ms; }

    // returns time of last time target was seen
    uint32_t last_backend_los_meas_ms() const { return _last_backend_los_meas_ms; }

    // vehicle has to be closer than this many m's to the target before descending towards target
    float get_max_xy_error_before_descending_m() const { return _xy_max_dist_desc_m; }

    // maximum correction during fine alignment phase (0 = unlimited)
    float get_fine_correction_max_m() const { return _fine_correction_max_m; }

    // returns orientation of sensor
    Rotation get_orient() const { return _orient; }

    // returns ekf outlier count
    uint32_t ekf_outlier_count() const { return _outlier_reject_count; }

    // give chance to driver to get updates from sensor, should be called at 400hz
    void update(float rangefinder_alt_m, bool rangefinder_alt_valid);

    // returns target position relative to the EKF origin
    bool get_target_position_m(Vector2p& ret);

    // returns target relative position as 3D vector
    void get_target_position_measurement_NED_m(Vector3f& ret);

    // returns target position relative to vehicle
    bool get_target_position_relative_NE_m(Vector2f& ret);

    // returns target velocity relative to vehicle
    bool get_target_velocity_relative_NE_ms(Vector2f& ret);

    // get the absolute velocity of the vehicle
    void get_target_velocity_ms(const Vector2f& vehicle_velocity_ms, Vector2f& target_vel_ms);

    // returns true when the landing target has been detected
    bool target_acquired();

    // returns true immediately when a backend measurement is present (without waiting for EKF initialization)
    bool target_visible() const;

    // process a LANDING_TARGET mavlink message
    void handle_msg(const mavlink_landing_target_t &packet, uint32_t timestamp_ms);

    // Get maximum yaw rate for alignment in degrees per second
    float get_yaw_rate_max_dps() const { return static_cast<float>(_yaw_rate_max_dps.get()); }
    
    // Get yaw stable time before descent continues
    float get_yaw_stable_time_s() const { return _yaw_stable_time_s.get(); }
    
    // Get acceptable XY error for slow descent
    float get_acceptable_error_m() const { return _acceptable_error_m.get(); }
    
    // Get minimum descent speed
    float get_min_descent_speed_ms() const { return _min_descent_speed_ms.get(); }
    
    // Get yaw XY gate setting
    bool yaw_xy_gate_enabled() const { return _yaw_xy_gate.get() != 0; }

    // State of the Landing Target Location
    enum class TargetState: uint8_t {
        TARGET_NEVER_SEEN = 0,
        TARGET_OUT_OF_RANGE,
        TARGET_RECENTLY_LOST,
        TARGET_FOUND
    };

    // ==========================================================================
    // Yaw Alignment State Machine
    // ==========================================================================
    enum class YawAlignState : uint8_t {
        DISABLED = 0,           // Yaw alignment disabled
        SEARCHING,              // Descending to find target
        COARSE_ALIGNING,        // Aligning yaw (no descent)
        COARSE_HOLDING,         // Holding coarse alignment
        DESCENDING,             // Normal descent with yaw correction
        FINE_ALIGNING,          // Fine alignment at low altitude
        FINE_HOLDING,           // Holding fine alignment
        FINAL_DESCENT           // Final descent with yaw maintenance
    };

    struct YawAlignResult {
        bool allow_descent;         // True if descent is permitted
        bool yaw_aligned;           // True if yaw is currently within tolerance
        float desired_yaw_rad;      // Target yaw angle in radians (NED frame)
        float yaw_error_deg;        // Current yaw error in degrees
        YawAlignState state;        // Current state
    };

    // return the last time PrecLand library had a output of the landing target position
    uint32_t get_last_valid_target_ms() const { return _last_valid_target_ms; }

    // return the current state of the location of the target
    TargetState get_target_state() const { return _current_target_state; }

    // return the last known landing position in Earth Frame NED meters.
    void get_last_detected_landing_pos_NED_m(Vector3p &pos) const { pos = _last_target_pos_rel_origin_ned_m; }

    // return the last known position of the vehicle when the target was detected in Earth Frame NED meters.
    void get_last_vehicle_pos_when_target_detected_NED_m(Vector3p &pos) const { pos = _last_vehicle_pos_ned_m; }

    // Parameter getters
    AC_PrecLand_StateMachine::RetryStrictness get_retry_strictness() const { return static_cast<AC_PrecLand_StateMachine::RetryStrictness>(_strict.get()); }
    uint8_t get_max_retry_allowed() const { return _retry_max; }
    float get_min_retry_time_sec() const { return _retry_timeout_s; }
    AC_PrecLand_StateMachine::RetryAction get_retry_behaviour() const { return static_cast<AC_PrecLand_StateMachine::RetryAction>(_retry_behave.get()); }

    bool allow_precland_after_reposition() const { return _options & PLND_OPTION_PRECLAND_AFTER_REPOSITION; }
    bool do_fast_descend() const { return _options & PLND_OPTION_FAST_DESCEND; }

    /*
      get target location lat/lon. Note that altitude in returned
      location is not reliable
     */
    bool get_target_location(Location &loc);

    /*
      get the absolute velocity of the target in m/s.
      return false if we cannot estimate target velocity or if the target is not acquired
    */
    bool get_target_velocity(Vector2f& ret);

    // ============================================================================
    // Target Yaw Orientation Support (for Precision Landing with Yaw-Kontrolle)
    // ============================================================================

    // get target yaw in radians (NED frame, 0 = North, positive = clockwise)
    // returns true if target yaw is valid and recent, false otherwise
    // yaw_rad is only modified if function returns true
    bool get_target_yaw_rad(float &yaw_rad) const;

    // returns true if we have a valid target yaw measurement
    bool target_yaw_valid() const;

    // get the raw target yaw without timeout check (for logging)
    float get_target_yaw_rad_raw() const { return _target_yaw_rad; }

    // get timestamp of last target yaw update
    uint32_t get_target_yaw_timestamp_ms() const { return _target_yaw_timestamp_ms; }

    // ==========================================================================
    // Yaw Alignment Control Interface
    // ==========================================================================
    void yaw_align_init();
    YawAlignResult yaw_align_update(float rangefinder_alt_m);
    YawAlignState get_yaw_align_state() const { return _yaw_align_state; }
    // FIX: yaw_align_enabled() should only check if the feature is configured, not current target status
    // -1 = disabled, any other value = enabled (0 means align to tag's X-axis)
    bool yaw_align_enabled() const { return _yaw_align_target_cd.get() != -1; }
    bool get_desired_yaw_for_alignment(float &yaw_rad) const;
    float get_yaw_alignment_error_rad() const;
    bool yaw_within_tolerance(float tolerance_cd) const;

    // ============================================================================

    // parameter var table
    static const struct AP_Param::GroupInfo var_info[];

private:
    enum class EstimatorType : uint8_t {
        RAW_SENSOR = 0,
        KALMAN_FILTER = 1,
    };

    // types of precision landing (used for PRECLAND_TYPE parameter)
    enum class Type : uint8_t {
        NONE = 0,
#if AC_PRECLAND_MAVLINK_ENABLED
        MAVLINK = 1,
#endif
#if AC_PRECLAND_IRLOCK_ENABLED
        IRLOCK = 2,
#endif
#if AC_PRECLAND_SITL_GAZEBO_ENABLED
        SITL_GAZEBO = 3,
#endif
#if AC_PRECLAND_SITL_ENABLED
        SITL = 4,
#endif
    };

    enum PLndOptions {
        PLND_OPTION_DISABLED = 0,
        PLND_OPTION_MOVING_TARGET = (1 << 0),
        PLND_OPTION_PRECLAND_AFTER_REPOSITION = (1 << 1),
        PLND_OPTION_FAST_DESCEND = (1 << 2),
    };

    // frames for vectors from vehicle to target
    enum class VectorFrame : uint8_t {
        BODY_FRD = 0,     // body frame, forward-right-down relative to the vehicle's attitude
        LOCAL_FRD = 1,    // forward-right-down where forward is aligned with front of the vehicle in the horizontal plane
    };

    // check the status of the target
    void check_target_status(float rangefinder_alt_m, bool rangefinder_alt_valid);

    // Check if the landing target is supposed to be in sight based on the height of the vehicle from the ground
    // This needs a valid rangefinder to work, if the min/max parameters are non zero
    bool check_if_sensor_in_range(float rangefinder_alt_m, bool rangefinder_alt_valid) const;

    // check if EKF got the time to initialize when the landing target was first detected
    // Expects sensor to update within EKF_INIT_SENSOR_MIN_UPDATE_MS milliseconds till EKF_INIT_TIME_MS milliseconds have passed
    // after this period landing target estimates can be used by vehicle
    void check_ekf_init_timeout();

    // run target position estimator
    void run_estimator(float rangefinder_alt_m, bool rangefinder_alt_valid);

    // If a new measurement was retrieved, sets _target_pos_rel_meas_ned_m and returns true
    bool construct_pos_meas_using_rangefinder(float rangefinder_alt_m, bool rangefinder_alt_valid);

    // get 3D vector from vehicle to target and frame.  returns true on success, false on failure
    bool retrieve_los_meas(Vector3f& target_vec_unit, VectorFrame& frame);

    // calculate target's position and velocity relative to the vehicle (used as input to position controller)
    // results are stored in_target_pos_rel_out_NE, _target_vel_rel_out_ne_ms
    void run_output_prediction();

    // ============================================================================
    // Target Yaw - Private setter (called by backends)
    // ============================================================================

    // set target yaw from quaternion received from landing target
    // called by backend when a valid quaternion is received
    // yaw_rad: target yaw in radians (NED frame)
    // timestamp_ms: timestamp when measurement was received
    void set_target_yaw_rad(float yaw_rad, uint32_t timestamp_ms);

    // ============================================================================

    // parameters
    AP_Int8                     _enabled;               // enabled/disabled
    AP_Enum<Type>               _type;                  // precision landing sensor type
    AP_Int8                     _bus;                   // which sensor bus
    AP_Enum<EstimatorType>      _estimator_type;        // precision landing estimator type
    AP_Float                    _lag_s;                 // sensor lag in seconds
    AP_Float                    _yaw_align_cd;          // Yaw angle from body x-axis to sensor x-axis.
    AP_Float                    _land_ofs_cm_x;         // Desired landing position of the camera forward of the target in vehicle body frame
    AP_Float                    _land_ofs_cm_y;         // Desired landing position of the camera right of the target in vehicle body frame
    AP_Float                    _accel_noise;           // accelerometer process noise
    AP_Vector3f                 _cam_offset_m;          // Position of the camera relative to the CG
    AP_Float                    _xy_max_dist_desc_m;    // Vehicle doing prec land will only descent vertically when horizontal error (in m) is below this limit
    AP_Int8                     _strict;                // PrecLand strictness
    AP_Int8                     _retry_max;             // PrecLand Maximum number of retires to a failed landing
    AP_Float                    _retry_timeout_s;       // Time for which vehicle continues descend even if target is lost. After this time period, vehicle will attempt a landing retry depending on PLND_STRICT param.
    AP_Int8                     _retry_behave;          // Action to do when trying a landing retry
    AP_Float                    _sensor_min_alt_m;      // PrecLand minimum height required for detecting target
    AP_Float                    _sensor_max_alt_m;      // PrecLand maximum height the sensor can detect target
    AP_Int16                    _options;               // Bitmask for extra options
    AP_Enum<Rotation>           _orient;                // Orientation of camera/sensor

    // ==========================================================================
    // Yaw Alignment Parameters
    // ==========================================================================
    AP_Int16                    _yaw_align_target_cd;   // Desired yaw offset from target (cdeg)
    AP_Int16                    _yaw_coarse_tol_cd;     // Coarse alignment tolerance (cdeg)
    AP_Int16                    _yaw_fine_tol_cd;       // Fine alignment tolerance (cdeg)
    AP_Float                    _yaw_hold_time_s;       // Hold time for alignment (s)
    AP_Float                    _yaw_fine_alt_m;        // Altitude for fine alignment (m)
    AP_Int8                     _yaw_rate_max_dps;      // Max yaw rate (deg/s)
    AP_Float                    _yaw_stable_time_s;     // Time yaw must be stable before descent continues (s)
    AP_Float                    _acceptable_error_m;    // Acceptable XY error for slow descent (m)
    AP_Float                    _min_descent_speed_ms;  // Minimum descent speed (m/s)
    AP_Int8                     _yaw_xy_gate;           // Gate: Yaw-align only if XY tolerance met (0=disabled, 1=enabled)
    AP_Float                    _yaw_max_alt_m;         // Maximum altitude for yaw alignment (m)
    AP_Float                    _yaw_filter_alpha;      // Yaw filter coefficient (0-1, higher = more filtering)
    AP_Float                    _fine_correction_max_m; // Max correction during fine align (m)

    uint32_t                    _last_update_ms;            // system time in millisecond when update was last called
    bool                        _target_acquired;           // true if target has been seen recently after estimator is initialized
    bool                        _estimator_initialized;     // true if estimator has been initialized after few seconds of the target being detected by sensor
    uint32_t                    _estimator_init_ms;         // system time in millisecond when EKF was init
    uint32_t                    _last_backend_los_meas_ms;  // system time target was last seen
    uint32_t                    _last_valid_target_ms;      // last time PrecLand library had a output of the landing target position

    PosVelEKF                   _ekf_x, _ekf_y;             // Kalman Filter for x and y axis
    uint32_t                    _outlier_reject_count;      // mini-EKF's outlier counter (3 consecutive outliers lead to EKF accepting updates)

    Vector3f                    _target_pos_rel_meas_ned_m; // target's relative position as 3D vector
    Vector3f                    _approach_vector_body;      // unit vector in landing approach direction (in body frame)

    Vector3p                    _last_target_pos_rel_origin_ned_m;  // stores the last known location of the target horizontally, and the height of the vehicle where it detected this target in meters NED
    Vector3p                    _last_vehicle_pos_ned_m;            // stores the position of the vehicle when landing target was last detected in m and NED
    Vector2f                    _target_pos_rel_est_ne_m;           // target's position relative to the IMU, not compensated for lag
    Vector2f                    _target_vel_rel_est_ne_ms;          // target's velocity relative to the IMU, not compensated for lag

    Vector2f                    _target_pos_rel_out_ne_m;   // target's position relative to the camera, fed into position controller
    Vector2f                    _target_vel_rel_out_ne_ms;  // target's velocity relative to the CG, fed into position controller
    Vector3f                    _last_veh_velocity_NED_ms;  // AHRS velocity at last estimate

    TargetState                 _current_target_state;      // Current status of the landing target

    // ============================================================================
    // Target Yaw Orientation Variables
    // ============================================================================
    float                       _target_yaw_rad;            // Target yaw orientation in radians (NED frame, 0=North, positive=clockwise)
    bool                        _target_yaw_valid;          // True if target yaw has been received and is recent
    uint32_t                    _target_yaw_timestamp_ms;   // System time in ms when target yaw was last updated

    // ==========================================================================
    // Yaw Alignment State Machine Variables
    // ==========================================================================
    YawAlignState               _yaw_align_state;           // Current yaw alignment state
    uint32_t                    _yaw_align_start_ms;        // Time when current state started
    uint32_t                    _yaw_hold_start_ms;         // Time when yaw entered tolerance
    uint32_t                    _yaw_stable_start_ms;       // Time when yaw became stable (for hover timer)
    bool                        _yaw_in_tolerance;          // True if currently within tolerance
    bool                        _coarse_align_complete;     // True after coarse alignment done
    float                       _last_desired_yaw_rad;      // Last commanded yaw for smooth transitions
    uint32_t                    _target_lost_timestamp_ms;  // Time since target was lost
    // ============================================================================

    // structure and buffer to hold a history of vehicle velocity
    struct inertial_data_frame_s {
        Matrix3f Tbn;                               // dcm rotation matrix to rotate body frame to north
        Vector3f correctedVehicleDeltaVelocityNED;
        Vector3f inertialNavVelocity;
        bool inertialNavVelocityValid;
        float dt;
        uint64_t time_usec;
    };
    ObjectArray<inertial_data_frame_s> *_inertial_history;
    struct inertial_data_frame_s *_inertial_data_delayed;

    // backend state
    struct precland_state {
        bool    healthy;
    } _backend_state;
    AC_PrecLand_Backend         *_backend;  // pointers to backend precision landing driver

    // write out PREC message to log:
    void Write_Precland();
    uint32_t _last_log_ms;  // last time we logged

    static AC_PrecLand *_singleton; //singleton
};

namespace AP {
    AC_PrecLand *ac_precland();
};

#endif // AC_PRECLAND_ENABLED