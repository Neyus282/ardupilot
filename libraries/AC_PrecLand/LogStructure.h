#pragma once

#include <AP_Logger/LogStructure.h>
#include "AC_PrecLand_config.h"

#define LOG_IDS_FROM_PRECLAND \
    LOG_PRECLAND_MSG, \
    LOG_PRECLAND2_MSG

// @LoggerMessage: PL
// @Description: Precision Landing messages
// @Field: TimeUS: Time since system startup
// @Field: Heal: True if Precision Landing is healthy
// @Field: TAcq: True if landing target is detected
// @Field: pX: Target position relative to vehicle, X-Axis (0 if target not found)
// @Field: pY: Target position relative to vehicle, Y-Axis (0 if target not found)
// @Field: vX: Target velocity relative to vehicle, X-Axis (0 if target not found)
// @Field: vY: Target velocity relative to vehicle, Y-Axis (0 if target not found)
// @Field: mX: Target's relative to origin position as 3-D Vector, X-Axis
// @Field: mY: Target's relative to origin position as 3-D Vector, Y-Axis
// @Field: mZ: Target's relative to origin position as 3-D Vector, Z-Axis
// @Field: LastMeasMS: Time when target was last detected
// @Field: EKFOutl: EKF's outlier count
// @Field: Est: Type of estimator used

// precision landing logging
struct PACKED log_Precland {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t healthy;
    uint8_t target_acquired;
    float pos_x;
    float pos_y;
    float vel_x;
    float vel_y;
    float meas_x;
    float meas_y;
    float meas_z;
    uint32_t last_meas;
    uint32_t ekf_outcount;
    uint8_t estimator;
};

// @LoggerMessage: PL2
// @Description: Precision Landing yaw alignment and diagnostics
// @Field: TimeUS: Time since system startup
// @Field: YSt: Yaw alignment state (0=Disabled,1=Searching,2=XYCenter,3=CoarseAlign,4=CoarseHold,5=Descend,6=FineAlign,7=FineHold,8=FinalDesc)
// @Field: TVis: True if target is visible (backend has recent measurement)
// @Field: TAcq: True if target is acquired (EKF initialized)
// @Field: TYaw: Target yaw orientation in degrees (NED frame)
// @Field: YErr: Current yaw alignment error in degrees
// @Field: MDt: Time since last sensor measurement in milliseconds
// @Field: RFAl: Rangefinder altitude in meters
// @Field: XYEr: Horizontal distance from vehicle to target in meters

struct PACKED log_PreclandYaw {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t yaw_state;
    uint8_t target_visible;
    uint8_t target_acquired;
    float target_yaw_deg;
    float yaw_error_deg;
    uint32_t meas_age_ms;
    float rf_alt_m;
    float xy_error_m;
};

#if AC_PRECLAND_ENABLED
#define LOG_STRUCTURE_FROM_PRECLAND                                     \
    { LOG_PRECLAND_MSG, sizeof(log_Precland),                           \
      "PL",    "QBBfffffffIIB",    "TimeUS,Heal,TAcq,pX,pY,vX,vY,mX,mY,mZ,LastMeasMS,EKFOutl,Est", "s--mmnnmmms--","F--0000000C--" , true }, \
    { LOG_PRECLAND2_MSG, sizeof(log_PreclandYaw),                       \
      "PL2",   "QBBBffIff",        "TimeUS,YSt,TVis,TAcq,TYaw,YErr,MDt,RFAl,XYEr", "s---ds-mm","F---00-00" , true },
#else
#define LOG_STRUCTURE_FROM_PRECLAND
#endif
