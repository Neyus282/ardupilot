# MAVLink 2 AprilTags Precision Landing - PRODUCTION VERSION
# OpenMV RT1062 + Pixhawk 6X + ArduPilot
# Version 5.3 - Based on working code + Multi-Marker support
# Grün = Tag erkannt, Rot = Kein Tag

import math, sensor, struct, time, machine

# LEDs
led_green = machine.Pin("LED_RED", machine.Pin.OUT)
led_red = machine.Pin("LED_GREEN", machine.Pin.OUT)

# BEIDE LEDS AUS beim Start
led_green.off()
led_red.off()
time.sleep(0.1)

# =============================================================================
# CONFIGURATION
# =============================================================================

uart_baudrate = 115200
MAV_system_id = 1
MAV_component_id = 0x54

lens_mm = 2.8
lens_to_camera_mm = 22
sensor_w_mm = 4.592  # OV5640
sensor_h_mm = 3.423  # OV5640

# Marker definitions: ID -> (size_mm, pos_x_mm, pos_y_mm)
# pos_x_mm: Forward distance from landing point (positive = marker is ahead)
# pos_y_mm: Right distance from landing point (positive = marker is right)
MARKERS = {
    0: (183,  121,   0),   # 121mm forward, centered
    1: (113,  -27,  35),   # 27mm behind, 35mm right
    2: (70,   -48, -56),   # 48mm behind, 56mm left
    3: (43,     9, -70),   # 9mm forward, 70mm left
    4: (27,     0, -35),   # at landing height, 35mm left
}

# Pre-compute marker data
MARKER_SIZES = {k: v[0] for k, v in MARKERS.items()}
MARKER_OFFSETS = {k: (-v[1], -v[2]) for k, v in MARKERS.items()}  # Negate for correction

# Marker switching hysteresis
HYSTERESIS = 1.2

# =============================================================================
# CAMERA SETUP
# =============================================================================

sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QQVGA)
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)

x_res = 160
y_res = 120
f_x = (lens_mm / sensor_w_mm) * x_res
f_y = (lens_mm / sensor_h_mm) * y_res
c_x = x_res / 2
c_y = y_res / 2
h_fov = 2 * math.atan((sensor_w_mm / 2) / lens_mm)
v_fov = 2 * math.atan((sensor_h_mm / 2) / lens_mm)

# =============================================================================
# FUNCTIONS
# =============================================================================

def z_to_mm(z_translation, tag_size):
    return (((z_translation * 100) * tag_size) / 165) - lens_to_camera_mm


uart = machine.UART(1, uart_baudrate, timeout_char=1000)
packet_sequence = 0


def checksum(data, extra):
    output = 0xFFFF
    for i in range(len(data)):
        tmp = data[i] ^ (output & 0xFF)
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        output = ((output >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    tmp = extra ^ (output & 0xFF)
    tmp = (tmp ^ (tmp << 4)) & 0xFF
    output = ((output >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return output


MAV_LANDING_TARGET_message_id = 149
MAV_FRAME_BODY_FRD = 12
MAV_LANDING_TARGET_extra_crc = 200


def send_landing_target_mavlink2(tag, tag_size, ofs_x, ofs_y):
    global packet_sequence

    # 3D Position berechnen
    x_mm = z_to_mm(tag.x_translation, tag_size)
    y_mm = z_to_mm(tag.y_translation, tag_size)
    z_mm = z_to_mm(tag.z_translation, tag_size)

    # Apply marker offset to get landing point position
    x_mm += ofs_y
    y_mm += ofs_x

    # Kamera-Frame zu Body-Frame (FRD)
    body_x = z_mm / 1000.0  # forward (m)
    body_y = x_mm / 1000.0  # right (m)
    body_z = y_mm / 1000.0  # down (m)

    # Winkel mit Offset-Korrektur
    TAN_H = math.tan(h_fov / 2)
    TAN_V = math.tan(v_fov / 2)

    if z_mm > 50:
        px_ofs_x = ofs_y * x_res / (z_mm * TAN_H * 2)
        px_ofs_y = -ofs_x * y_res / (z_mm * TAN_V * 2)
    else:
        px_ofs_x = px_ofs_y = 0

    lp_cx = tag.cx + px_ofs_x
    lp_cy = tag.cy + px_ofs_y

    angle_x = ((lp_cx / x_res) - 0.5) * h_fov
    angle_y = ((lp_cy / y_res) - 0.5) * v_fov
    distance = math.sqrt(body_x**2 + body_y**2 + body_z**2)

    size_x = (tag.w / x_res) * h_fov
    size_y = (tag.h / y_res) * v_fov

    # Yaw from tag rotation
    yaw = tag.z_rotation
    while yaw > 3.14159265:
        yaw -= 6.28318530
    while yaw < -3.14159265:
        yaw += 6.28318530
    hy = yaw / 2.0
    qw, qx, qy, qz = math.cos(hy), 0.0, 0.0, math.sin(hy)

    # MAVLink 2 Payload (60 bytes)
    payload = struct.pack(
        "<QfffffBBfffffffBB",
        0,  # time_usec
        angle_x,  # angle_x (rad)
        angle_y,  # angle_y (rad)
        distance,  # distance (m)
        size_x,  # size_x (rad)
        size_y,  # size_y (rad)
        tag.id,  # target_num
        MAV_FRAME_BODY_FRD,  # frame
        body_x,  # x (m)
        body_y,  # y (m)
        body_z,  # z (m)
        qw,  # q[0]
        qx,  # q[1]
        qy,  # q[2]
        qz,  # q[3]
        0,  # type
        1,  # position_valid = 1
    )

    # MAVLink 2 Header (9 bytes for CRC calculation)
    msg_len = len(payload)
    msgid_bytes = struct.pack("<I", MAV_LANDING_TARGET_message_id)[:3]

    header_for_crc = struct.pack(
        "<BBBBBBB",
        msg_len,
        0,  # incompat_flags
        0,  # compat_flags
        packet_sequence & 0xFF,
        MAV_system_id,
        MAV_component_id,
        msgid_bytes[0],
    ) + msgid_bytes[1:3]

    # CRC
    crc = checksum(header_for_crc + payload, MAV_LANDING_TARGET_extra_crc)

    # Komplettes Paket
    packet = struct.pack("<B", 0xFD) + header_for_crc + payload + struct.pack("<H", crc)

    uart.write(packet)
    packet_sequence += 1

    return {"x": body_x, "y": body_y, "z": body_z, "dist": distance, "yaw": math.degrees(yaw)}


# Marker selection with hysteresis
current_id = None

def select_marker(tags):
    global current_id

    if not tags:
        return None, None, None, None

    valid = []
    for t in tags:
        if t.id in MARKER_SIZES:
            valid.append((t, t.w * t.h))

    if not valid:
        return None, None, None, None

    valid.sort(key=lambda x: x[1], reverse=True)
    best_tag, best_area = valid[0]

    if current_id is not None:
        for t, area in valid:
            if t.id == current_id:
                if best_tag.id != current_id and best_area > area * HYSTERESIS:
                    current_id = best_tag.id
                else:
                    size = MARKER_SIZES[current_id]
                    ofs = MARKER_OFFSETS[current_id]
                    return t, size, ofs[0], ofs[1]
                break
        else:
            current_id = best_tag.id
    else:
        current_id = best_tag.id

    size = MARKER_SIZES[current_id]
    ofs = MARKER_OFFSETS[current_id]
    return best_tag, size, ofs[0], ofs[1]


# =============================================================================
# MAIN LOOP
# =============================================================================

clock = time.clock()
print("=" * 50)
print("MAVLink 2 Precision Landing v5.3")
print("Multi-Marker with Yaw")
print("Tags:", list(MARKER_SIZES.keys()))
print("FOV: %.1f° x %.1f°" % (math.degrees(h_fov), math.degrees(v_fov)))
print("=" * 50)

msg_count = 0

while True:
    clock.tick()
    img = sensor.snapshot()

    tags = img.find_apriltags(fx=f_x, fy=f_y, cx=c_x, cy=c_y)

    # Select best marker with hysteresis
    tag, size, ofs_x, ofs_y = select_marker(tags)

    if tag is not None:
        mav = send_landing_target_mavlink2(tag, size, ofs_x, ofs_y)

        img.draw_rectangle(tag.rect)
        img.draw_cross(tag.cx, tag.cy)

        led_green.on()
        led_red.off()

        msg_count += 1
        if msg_count % 10 == 0:  # Print every 10th message
            print(
                "[%04d] ID:%d | x:%+.2f y:%+.2f z:%+.2f | yaw:%+.1f° | %.1ffps"
                % (msg_count, tag.id, mav["x"], mav["y"], mav["z"], mav["yaw"], clock.fps())
            )
    else:
        led_green.off()
        led_red.on()
