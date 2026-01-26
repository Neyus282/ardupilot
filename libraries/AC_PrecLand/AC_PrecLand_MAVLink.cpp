#include "AC_PrecLand_config.h"

#if AC_PRECLAND_MAVLINK_ENABLED

#include "AC_PrecLand_MAVLink.h"
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>

// perform any required initialisation of backend
void AC_PrecLand_MAVLink::init()
{
    // set healthy
    _state.healthy = true;
}

// retrieve updates from sensor
void AC_PrecLand_MAVLink::update()
{
    _los_meas.valid = _los_meas.valid && AP_HAL::millis() - _los_meas.time_ms <= 1000;
}

void AC_PrecLand_MAVLink::handle_msg(const mavlink_landing_target_t &packet, uint32_t timestamp_ms)
{
    // check frame is supported
    if (packet.frame != MAV_FRAME_BODY_FRD && packet.frame != MAV_FRAME_LOCAL_FRD) {
        if (!_wrong_frame_msg_sent) {
            _wrong_frame_msg_sent = true;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO,"Plnd: Frame not supported");
        }
        return;
    }

    // ==========================================================================
    // FIX: ALWAYS compute unit vector from ANGLES (angle_x, angle_y)
    // The angles are INDEPENDENT of tag size and are always correct!
    // The position (x, y, z) and distance depend on tag size which is often wrong.
    //
    // Angle convention for downward-facing camera:
    //   angle_x = horizontal offset (positive = target is to the right)
    //   angle_y = forward offset (positive = target is ahead)
    //
    // Body FRD convention:
    //   X = forward, Y = right, Z = down
    //
    // Unit vector towards target: (tan(angle_y), tan(angle_x), 1) normalized
    // ==========================================================================
    _los_meas.vec_unit = Vector3f{tanf(packet.angle_y), tanf(packet.angle_x), 1.0f};
    _los_meas.vec_unit.normalize();
    _los_meas.frame = (packet.frame == MAV_FRAME_BODY_FRD) ? AC_PrecLand::VectorFrame::BODY_FRD : AC_PrecLand::VectorFrame::LOCAL_FRD;

    // Don't use backend distance - it's based on tag size which may be wrong
    // Let the main code use rangefinder instead
    _distance_to_target = 0;  // 0 = use rangefinder
    
    _los_meas.time_ms = timestamp_ms;
    _los_meas.valid = true;
}

#endif // AC_PRECLAND_MAVLINK_ENABLED
