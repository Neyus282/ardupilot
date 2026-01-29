#!/usr/bin/env python3
"""
AprilTag MAVLink Simulator for Gazebo SITL - VERSION 3
=======================================================
Based on working V2, with added YAW CALIBRATION features:

NEW IN V3:
1. --invert-yaw flag to flip yaw sign
2. --yaw-offset to add constant offset (degrees)
3. Statistics tracking (mean, stddev) for yaw calibration
4. Clear raw_yaw vs sent_yaw in debug output
5. Better visual feedback

Coordinate Systems:
- AprilTag Library: X=right, Y=down, Z=forward (camera looking at tag)
- Downward Camera:  Looking down at ground
- ArduPilot Body FRD: X=forward, Y=right, Z=down

Author: Dominic (Precision Landing Yaw Control Project)
"""

import cv2
import math
import numpy as np
import time
import sys
import struct
import subprocess
import threading
import queue
from dataclasses import dataclass, field
from typing import Optional, Tuple, Dict, List
from collections import deque

# MAVLink
from pymavlink import mavutil

# AprilTag detection
try:
    from pupil_apriltags import Detector as AprilTagDetector
    APRILTAG_BACKEND = "pupil_apriltags"
except ImportError:
    try:
        from dt_apriltags import Detector as AprilTagDetector
        APRILTAG_BACKEND = "dt_apriltags"
    except ImportError:
        print("ERROR: No AprilTag library found!")
        sys.exit(1)


# =============================================================================
# MAVLink2 Manual Encoding (unchanged)
# =============================================================================

MAV_LANDING_TARGET_MSG_ID = 149
MAV_LANDING_TARGET_CRC_EXTRA = 200
MAV_FRAME_BODY_FRD = 12


def crc_accumulate(byte, crc):
    tmp = byte ^ (crc & 0xFF)
    tmp = (tmp ^ (tmp << 4)) & 0xFF
    return ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF


def crc_calculate(data, crc_extra):
    crc = 0xFFFF
    for byte in data:
        crc = crc_accumulate(byte, crc)
    crc = crc_accumulate(crc_extra, crc)
    return crc


def encode_landing_target_mavlink2(
    sequence: int,
    source_system: int,
    source_component: int,
    angle_x: float,
    angle_y: float,
    distance: float,
    size_x: float,
    size_y: float,
    target_num: int,
    frame: int,
    x: float,
    y: float,
    z: float,
    q_w: float,
    q_x: float,
    q_y: float,
    q_z: float,
    landing_type: int = 0,
    position_valid: int = 1
) -> bytes:
    """Encode complete MAVLink2 LANDING_TARGET message (60 bytes payload)."""
    
    payload = struct.pack(
        "<QfffffBBfffffffBB",
        0,                    # time_usec
        angle_x, angle_y, distance, size_x, size_y,
        target_num, frame,
        x, y, z,
        q_w, q_x, q_y, q_z,
        landing_type, position_valid
    )
    
    header = struct.pack(
        "<BBBBBBBBBB",
        0xFD, len(payload), 0, 0,
        sequence & 0xFF,
        source_system, source_component,
        MAV_LANDING_TARGET_MSG_ID & 0xFF,
        (MAV_LANDING_TARGET_MSG_ID >> 8) & 0xFF,
        (MAV_LANDING_TARGET_MSG_ID >> 16) & 0xFF,
    )
    
    crc = crc_calculate(header[1:] + payload, MAV_LANDING_TARGET_CRC_EXTRA)
    return header + payload + struct.pack("<H", crc)


# =============================================================================
# Configuration
# =============================================================================

@dataclass
class Config:
    sitl_host: str = "127.0.0.1"
    sitl_port: int = 5762
    
    mav_system_id: int = 255
    mav_component_id: int = 196
    
    gstreamer_port: int = 5600
    
    image_width: int = 640
    image_height: int = 480
    camera_hfov_deg: float = 80.0
    
    tag_family: str = "tag36h11"
    
    # Tag sizes for GAZEBO (default 0.8m = 800mm)
    valid_tag_ids: Dict[int, float] = None
    
    # Display options
    show_video: bool = True
    debug_print: bool = True
    debug_yaw: bool = True
    
    # ==========================================================================
    # NEW V3: Yaw calibration options
    # ==========================================================================
    invert_yaw: bool = False          # Flip yaw sign
    yaw_offset_deg: float = 0.0       # Add constant offset (degrees)
    
    def __post_init__(self):
        if self.valid_tag_ids is None:
            self.valid_tag_ids = {
                0: 800,   # 0.8m - Standard Gazebo AprilTag
                1: 800,
                2: 800,
            }
        
        self.camera_hfov_rad = math.radians(self.camera_hfov_deg)
        self.camera_vfov_rad = self.camera_hfov_rad * (self.image_height / self.image_width)
        
        self.fx = self.image_width / (2 * math.tan(self.camera_hfov_rad / 2))
        self.fy = self.fx
        self.cx = self.image_width / 2
        self.cy = self.image_height / 2
        
        self.yaw_offset_rad = math.radians(self.yaw_offset_deg)


# =============================================================================
# Yaw Statistics Tracker (NEW V3)
# =============================================================================

class YawStats:
    """Track yaw statistics for calibration."""
    
    def __init__(self, window_size: int = 100):
        self.window_size = window_size
        self.raw_yaw_history: deque = deque(maxlen=window_size)
        self.sent_yaw_history: deque = deque(maxlen=window_size)
        self.last_stats_time = time.time()
        self.stats_interval = 5.0  # Print stats every 5 seconds
    
    def add_sample(self, raw_yaw_deg: float, sent_yaw_deg: float):
        self.raw_yaw_history.append(raw_yaw_deg)
        self.sent_yaw_history.append(sent_yaw_deg)
    
    def get_stats(self) -> Tuple[float, float, float, float]:
        """Returns (raw_mean, raw_std, sent_mean, sent_std) in degrees."""
        if len(self.raw_yaw_history) < 2:
            return (0.0, 0.0, 0.0, 0.0)
        
        raw_arr = np.array(self.raw_yaw_history)
        sent_arr = np.array(self.sent_yaw_history)
        
        return (
            np.mean(raw_arr),
            np.std(raw_arr),
            np.mean(sent_arr),
            np.std(sent_arr)
        )
    
    def should_print_stats(self) -> bool:
        if time.time() - self.last_stats_time >= self.stats_interval:
            self.last_stats_time = time.time()
            return True
        return False
    
    def print_stats(self):
        raw_mean, raw_std, sent_mean, sent_std = self.get_stats()
        n = len(self.raw_yaw_history)
        print(f"\n{'='*60}")
        print(f"YAW STATISTICS (last {n} samples)")
        print(f"  Raw Yaw:  mean={raw_mean:+7.2f}°  std={raw_std:5.2f}°")
        print(f"  Sent Yaw: mean={sent_mean:+7.2f}°  std={sent_std:5.2f}°")
        print(f"{'='*60}\n")


# =============================================================================
# AprilTag Detection Result
# =============================================================================

@dataclass
class TagDetection:
    id: int
    center: Tuple[float, float]
    corners: np.ndarray
    pose_R: Optional[np.ndarray]
    pose_t: Optional[np.ndarray]
    tag_size_mm: float
    
    @property
    def cx(self) -> float:
        return self.center[0]
    
    @property
    def cy(self) -> float:
        return self.center[1]
    
    @property
    def width(self) -> float:
        return np.linalg.norm(self.corners[1] - self.corners[0])
    
    @property
    def height(self) -> float:
        return np.linalg.norm(self.corners[2] - self.corners[1])


# =============================================================================
# Main Simulator Class
# =============================================================================

class AprilTagMAVLinkSim:
    def __init__(self, config: Config):
        self.config = config
        self.packet_sequence = 0
        self.msg_count = 0
        self.mav_conn = None
        self.detector = None
        self.cap = None
        self.use_subprocess = False
        self.frame_queue = None
        self.capture_process = None
        self.capture_thread = None
        self.connected = False
        
        # V3: Yaw statistics
        self.yaw_stats = YawStats()
        
    def setup(self) -> bool:
        print("=" * 60)
        print("AprilTag MAVLink Simulator - VERSION 3 (Yaw Calibration)")
        print(f"AprilTag Backend: {APRILTAG_BACKEND}")
        print("=" * 60)
        
        # V3: Show calibration settings
        print(f"Yaw Settings:")
        print(f"  Invert Yaw: {self.config.invert_yaw}")
        print(f"  Yaw Offset: {self.config.yaw_offset_deg}°")
        print("=" * 60)
        
        if not self._setup_apriltag():
            return False
        if not self._setup_video():
            return False
        if not self._setup_mavlink():
            return False
        
        print("=" * 60)
        print("Setup complete!")
        print(f"Tag sizes (mm): {self.config.valid_tag_ids}")
        print("=" * 60)
        return True
    
    def _setup_apriltag(self) -> bool:
        try:
            self.detector = AprilTagDetector(
                families=self.config.tag_family,
                nthreads=4,
                quad_decimate=1.0,
                quad_sigma=0.0,
                refine_edges=1,
                decode_sharpening=0.25,
            )
            print(f"[OK] AprilTag detector initialized")
            return True
        except Exception as e:
            print(f"[ERROR] AprilTag init failed: {e}")
            return False
    
    def _setup_video(self) -> bool:
        print(f"[INFO] Opening video on port {self.config.gstreamer_port}...")
        
        gst_pipeline = (
            f"udpsrc port={self.config.gstreamer_port} "
            f"caps=\"application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264\" ! "
            f"rtph264depay ! avdec_h264 ! videoconvert ! video/x-raw,format=BGR ! appsink"
        )
        
        try:
            self.cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
            if self.cap.isOpened():
                print("[OK] GStreamer capture initialized")
                return True
        except:
            pass
        
        try:
            self.cap = cv2.VideoCapture(
                f"udp://127.0.0.1:{self.config.gstreamer_port}",
                cv2.CAP_FFMPEG
            )
            if self.cap.isOpened():
                print("[OK] FFmpeg capture initialized")
                return True
        except:
            pass
        
        print("[ERROR] Video capture failed!")
        return False
    
    def _setup_mavlink(self) -> bool:
        connection_string = f"tcp:{self.config.sitl_host}:{self.config.sitl_port}"
        print(f"[INFO] Connecting to SITL: {connection_string}")
        
        try:
            self.mav_conn = mavutil.mavlink_connection(
                connection_string,
                source_system=self.config.mav_system_id,
                source_component=self.config.mav_component_id
            )
            
            print("[INFO] Waiting for heartbeat...")
            msg = self.mav_conn.wait_heartbeat(timeout=30)
            
            if msg is None:
                print("[ERROR] No heartbeat!")
                return False
            
            self.connected = True
            print(f"[OK] Connected! Target: {self.mav_conn.target_system}")
            return True
            
        except Exception as e:
            print(f"[ERROR] Connection failed: {e}")
            return False
    
    def detect_tags(self, frame: np.ndarray) -> list:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        camera_params = (self.config.fx, self.config.fy, self.config.cx, self.config.cy)
        
        detections = []
        results = self.detector.detect(gray, estimate_tag_pose=True,
                                       camera_params=camera_params, tag_size=0.1)
        
        for r in results:
            if r.tag_id in self.config.valid_tag_ids:
                tag_size_mm = self.config.valid_tag_ids[r.tag_id]
                tag_size_m = tag_size_mm / 1000.0
                
                results_sized = self.detector.detect(
                    gray, estimate_tag_pose=True,
                    camera_params=camera_params, tag_size=tag_size_m
                )
                
                for rs in results_sized:
                    if rs.tag_id == r.tag_id:
                        detections.append(TagDetection(
                            id=rs.tag_id,
                            center=rs.center,
                            corners=rs.corners,
                            pose_R=rs.pose_R,
                            pose_t=rs.pose_t,
                            tag_size_mm=tag_size_mm
                        ))
                        break
        
        detections.sort(key=lambda d: d.width * d.height, reverse=True)
        return detections
    
    # =========================================================================
    # Yaw extraction for DOWNWARD-FACING camera
    # =========================================================================
    
    def extract_yaw_for_downward_camera(self, R: np.ndarray) -> float:
        """
        Extract yaw from rotation matrix for a DOWNWARD-FACING camera.
        Returns RAW yaw before any calibration adjustments.
        """
        # Die X-Achse des Tags im Kamera-Frame
        tag_x_in_cam = R[:, 0]
        
        # Projiziere auf Kamera XY-Ebene und berechne Winkel
        yaw_cam = math.atan2(tag_x_in_cam[1], tag_x_in_cam[0])
        
        # Konvertiere zu ArduPilot Konvention (clockwise positive)
        yaw_ardupilot = yaw_cam
        
        return yaw_ardupilot
    
    def apply_yaw_calibration(self, raw_yaw_rad: float) -> float:
        """
        Apply calibration adjustments to raw yaw.
        V3 NEW: Supports inversion and offset.
        """
        yaw = raw_yaw_rad
        
        # Apply inversion if configured
        if self.config.invert_yaw:
            yaw = -yaw
        
        # Apply offset
        yaw += self.config.yaw_offset_rad
        
        # Wrap to [-π, π]
        while yaw > math.pi:
            yaw -= 2 * math.pi
        while yaw < -math.pi:
            yaw += 2 * math.pi
        
        return yaw
    
    def yaw_to_quaternion(self, yaw_rad: float) -> Tuple[float, float, float, float]:
        """Convert yaw angle to quaternion [w, x, y, z] for pure Z rotation."""
        half = yaw_rad / 2.0
        return (math.cos(half), 0.0, 0.0, math.sin(half))
    
    def send_landing_target(self, tag: TagDetection, frame_shape: Tuple[int, int]) -> dict:
        """Send LANDING_TARGET with calibrated yaw."""
        h, w = frame_shape[:2]
        
        # Position
        if tag.pose_t is not None:
            x_cam = tag.pose_t[0][0]
            y_cam = tag.pose_t[1][0]
            z_cam = tag.pose_t[2][0]
            
            # Camera → Body frame (downward-facing)
            body_x = z_cam
            body_y = x_cam
            body_z = y_cam
        else:
            body_x, body_y, body_z = 0.0, 0.0, 1.0
        
        distance = math.sqrt(body_x**2 + body_y**2 + body_z**2)
        
        # Angles
        # angle_x: horizontal offset (positive = target is to the RIGHT)
        # angle_y: forward offset (positive = target is AHEAD/FORWARD)
        #
        # For downward-facing camera:
        # - Image X increases to the right → matches body Y (right)
        # - Image Y increases downward, but "ahead" is at TOP of image
        #   So we NEGATE angle_y to get correct forward direction
        angle_x = ((tag.cx / w) - 0.5) * self.config.camera_hfov_rad
        angle_y = -((tag.cy / h) - 0.5) * self.config.camera_vfov_rad  # NEGATED!
        
        size_x = (tag.width / w) * self.config.camera_hfov_rad
        size_y = (tag.height / h) * self.config.camera_vfov_rad
        
        # =====================================================================
        # V3: Yaw extraction with calibration
        # =====================================================================
        raw_yaw_rad = 0.0
        sent_yaw_rad = 0.0
        
        if tag.pose_R is not None:
            # Get raw yaw from rotation matrix
            raw_yaw_rad = self.extract_yaw_for_downward_camera(tag.pose_R)
            
            # Apply calibration (inversion, offset)
            sent_yaw_rad = self.apply_yaw_calibration(raw_yaw_rad)
            
            # Debug: Zeige die Rotationsmatrix (nur alle 30 Frames)
            if self.config.debug_yaw and self.msg_count % 30 == 0:
                print(f"\n[DEBUG] Rotation Matrix R:")
                print(f"  [{tag.pose_R[0,0]:+.3f} {tag.pose_R[0,1]:+.3f} {tag.pose_R[0,2]:+.3f}]")
                print(f"  [{tag.pose_R[1,0]:+.3f} {tag.pose_R[1,1]:+.3f} {tag.pose_R[1,2]:+.3f}]")
                print(f"  [{tag.pose_R[2,0]:+.3f} {tag.pose_R[2,1]:+.3f} {tag.pose_R[2,2]:+.3f}]")
                print(f"[DEBUG] Raw Yaw: {math.degrees(raw_yaw_rad):+.1f}° → Sent Yaw: {math.degrees(sent_yaw_rad):+.1f}°")
        
        # V3: Track statistics
        self.yaw_stats.add_sample(math.degrees(raw_yaw_rad), math.degrees(sent_yaw_rad))
        
        # Quaternion from calibrated yaw
        q_w, q_x, q_y, q_z = self.yaw_to_quaternion(sent_yaw_rad)
        
        # Send
        packet = encode_landing_target_mavlink2(
            sequence=self.packet_sequence,
            source_system=self.config.mav_system_id,
            source_component=self.config.mav_component_id,
            angle_x=angle_x, angle_y=angle_y,
            distance=distance,
            size_x=size_x, size_y=size_y,
            target_num=tag.id,
            frame=MAV_FRAME_BODY_FRD,
            x=body_x, y=body_y, z=body_z,
            q_w=q_w, q_x=q_x, q_y=q_y, q_z=q_z,
            landing_type=0, position_valid=1
        )
        
        try:
            self.mav_conn.write(packet)
        except Exception as e:
            print(f"[ERROR] Send failed: {e}")
        
        self.packet_sequence += 1
        self.msg_count += 1
        
        return {
            "x": body_x, "y": body_y, "z": body_z,
            "dist": distance,
            "angle_x_deg": math.degrees(angle_x),
            "angle_y_deg": math.degrees(angle_y),
            "raw_yaw_deg": math.degrees(raw_yaw_rad),
            "sent_yaw_deg": math.degrees(sent_yaw_rad),
            "q": (q_w, q_x, q_y, q_z)
        }
    
    def draw_detection(self, frame: np.ndarray, tag: TagDetection, mav_data: dict):
        corners = tag.corners.astype(int)
        cv2.polylines(frame, [corners], True, (0, 255, 0), 2)
        
        cx, cy = int(tag.cx), int(tag.cy)
        cv2.drawMarker(frame, (cx, cy), (0, 255, 0), cv2.MARKER_CROSS, 20, 2)
        
        # Zeichne Yaw-Richtung als Pfeile
        line_len = 50
        
        # Raw yaw (blau, dünn)
        raw_yaw_rad = math.radians(mav_data['raw_yaw_deg'])
        raw_end_x = int(cx + line_len * math.sin(raw_yaw_rad))
        raw_end_y = int(cy - line_len * math.cos(raw_yaw_rad))
        cv2.arrowedLine(frame, (cx, cy), (raw_end_x, raw_end_y), (255, 100, 0), 1)
        
        # Sent yaw (rot, dick)
        sent_yaw_rad = math.radians(mav_data['sent_yaw_deg'])
        sent_end_x = int(cx + line_len * math.sin(sent_yaw_rad))
        sent_end_y = int(cy - line_len * math.cos(sent_yaw_rad))
        cv2.arrowedLine(frame, (cx, cy), (sent_end_x, sent_end_y), (0, 0, 255), 2)
        
        # V3: Show both raw and sent yaw
        lines = [
            f"ID: {tag.id}",
            f"Pos: ({mav_data['x']:.2f}, {mav_data['y']:.2f}, {mav_data['z']:.2f})",
            f"Raw:  {mav_data['raw_yaw_deg']:+.1f}deg",
            f"Sent: {mav_data['sent_yaw_deg']:+.1f}deg",
        ]
        
        y = cy - 70
        for line in lines:
            cv2.putText(frame, line, (cx - 80, y), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            y += 18
        
        return frame
    
    def run(self):
        fps_time = time.time()
        fps_count = 0
        fps = 0.0
        
        print("\n[INFO] Running. Press 'q' to quit.")
        print("[INFO] Yaw Calibration Tips:")
        print("  1. Place tag at known angle (e.g., 0° = North edge pointing North)")
        print("  2. Check 'Raw Yaw' value - should match physical angle")
        print("  3. If inverted: restart with --invert-yaw")
        print("  4. If offset: restart with --yaw-offset <degrees>")
        print("")
        print("[INFO] MAVProxy commands:")
        print("  mode GUIDED")
        print("  arm throttle")
        print("  takeoff 5")
        print("  mode LAND")
        print("")
        
        try:
            while True:
                ret, frame = self.cap.read()
                if not ret:
                    time.sleep(0.1)
                    continue
                
                tags = self.detect_tags(frame)
                
                if tags:
                    tag = tags[0]
                    mav_data = self.send_landing_target(tag, frame.shape)
                    
                    if self.config.debug_print:
                        # V3: Show angles and yaw
                        print(
                            f"[{self.msg_count:04d}] ID:{tag.id} | "
                            f"ang:({mav_data['angle_x_deg']:+5.1f}°,{mav_data['angle_y_deg']:+5.1f}°) | "
                            f"yaw:{mav_data['sent_yaw_deg']:+6.1f}° | "
                            f"{fps:.1f}fps"
                        )
                    
                    # V3: Print statistics periodically
                    if self.yaw_stats.should_print_stats():
                        self.yaw_stats.print_stats()
                    
                    if self.config.show_video:
                        frame = self.draw_detection(frame, tag, mav_data)
                
                fps_count += 1
                if time.time() - fps_time >= 1.0:
                    fps = fps_count / (time.time() - fps_time)
                    fps_count = 0
                    fps_time = time.time()
                
                if self.config.show_video:
                    # V3: Show calibration status
                    inv_str = "INV" if self.config.invert_yaw else ""
                    ofs_str = f"OFS:{self.config.yaw_offset_deg:+.0f}" if self.config.yaw_offset_deg != 0 else ""
                    cal_str = f" [{inv_str} {ofs_str}]" if inv_str or ofs_str else ""
                    
                    status = f"FPS:{fps:.1f} | Tags:{len(tags)} | Msgs:{self.msg_count}{cal_str}"
                    cv2.putText(frame, status, (10, 25), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                    
                    # Legend
                    cv2.putText(frame, "Blue=Raw, Red=Sent", (10, 45),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
                    
                    cv2.imshow("AprilTag V3 (Yaw Calibration)", frame)
                    
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
        
        except KeyboardInterrupt:
            print("\n[INFO] Interrupted")
        
        finally:
            # Print final statistics
            print("\n" + "="*60)
            print("FINAL YAW STATISTICS")
            self.yaw_stats.print_stats()
            
            if self.cap:
                self.cap.release()
            cv2.destroyAllWindows()
            if self.mav_conn:
                self.mav_conn.close()


# =============================================================================
# Entry Point
# =============================================================================

def main():
    import argparse
    
    parser = argparse.ArgumentParser(
        description="AprilTag MAVLink Sim V3 - with Yaw Calibration",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Yaw Calibration Examples:
  %(prog)s                          # Default settings
  %(prog)s --invert-yaw             # If drone turns wrong direction
  %(prog)s --yaw-offset 90          # If 90° offset needed
  %(prog)s --invert-yaw --yaw-offset -45  # Both adjustments

How to Calibrate:
  1. Place AprilTag with known orientation (e.g., top edge pointing North)
  2. Run script and observe 'Raw Yaw' value
  3. Expected: Tag at 0° should show Raw Yaw ≈ 0°
  4. If Raw Yaw shows -X° instead of +X°: use --invert-yaw
  5. If Raw Yaw shows Y° offset: use --yaw-offset -Y
        """
    )
    
    parser.add_argument("--port", type=int, default=5600,
                       help="GStreamer UDP port (default: 5600)")
    parser.add_argument("--sitl-port", type=int, default=5762,
                       help="SITL TCP port (default: 5762)")
    parser.add_argument("--tag-size", type=int, default=150,
                       help="Tag size in mm (default: 150 for Gazebo)")
    parser.add_argument("--no-video", action="store_true",
                       help="Disable video display")
    parser.add_argument("--no-debug", action="store_true",
                       help="Disable debug output")
    
    # V3: Yaw calibration options
    parser.add_argument("--invert-yaw", action="store_true",
                       help="Invert yaw sign (use if drone turns wrong direction)")
    parser.add_argument("--yaw-offset", type=float, default=0.0,
                       help="Add yaw offset in degrees (default: 0)")
    
    args = parser.parse_args()
    
    config = Config(
        gstreamer_port=args.port,
        sitl_port=args.sitl_port,
        show_video=not args.no_video,
        debug_print=not args.no_debug,
        debug_yaw=not args.no_debug,
        valid_tag_ids={0: args.tag_size, 1: args.tag_size, 2: args.tag_size},
        invert_yaw=args.invert_yaw,
        yaw_offset_deg=args.yaw_offset,
    )
    
    sim = AprilTagMAVLinkSim(config)
    
    if sim.setup():
        sim.run()
    else:
        sys.exit(1)


if __name__ == "__main__":
    main()