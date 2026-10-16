#!/usr/bin/env python3
# Records 2 minutes of RGB video from two ROS2 image topics to MP4 files.

# import rclpy
# from rclpy.node import Node
# from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
# import cv2
# import threading

# class DualRealSenseRecorder(Node):
#     def __init__(self):
#         super().__init__('dual_realsense_recorder')

#         # ---------- Parameters (override with --ros-args -p name:=value) ----------
#         self.declare_parameter('front_topic', '/front/cam2/color/image_raw')
#         self.declare_parameter('top_topic',   '/top/cam1/color/image_raw')
#         self.declare_parameter('front_output', 'front_rgb.mp4')
#         self.declare_parameter('top_output',   'top_rgb.mp4')
#         self.declare_parameter('fps', 30.0)              # metadata FPS; 30 is standard for RealSense RGB
#         self.declare_parameter('duration_sec', 60)      # 2 minutes
#         self.declare_parameter('fourcc', 'mp4v')         # 'mp4v' is widely supported; try 'avc1' if you have H.264

#         self.front_topic = self.get_parameter('front_topic').get_parameter_value().string_value
#         self.top_topic   = self.get_parameter('top_topic').get_parameter_value().string_value
#         self.front_out   = self.get_parameter('front_output').get_parameter_value().string_value
#         self.top_out     = self.get_parameter('top_output').get_parameter_value().string_value
#         self.fps         = float(self.get_parameter('fps').get_parameter_value().double_value)
#         self.duration    = int(self.get_parameter('duration_sec').get_parameter_value().integer_value)
#         self.fourcc_str  = self.get_parameter('fourcc').get_parameter_value().string_value

#         # ---------- CV / Writers ----------
#         self.bridge = CvBridge()
#         self.front_writer = None
#         self.top_writer = None
#         self.front_lock = threading.Lock()
#         self.top_lock = threading.Lock()

#         # ---------- QoS for sensor streams ----------
#         qos = QoSProfile(
#             reliability=QoSReliabilityPolicy.BEST_EFFORT,
#             history=QoSHistoryPolicy.KEEP_LAST,
#             depth=5
#         )

#         # ---------- Subscriptions ----------
#         self.front_sub = self.create_subscription(Image, self.front_topic, self.front_cb, qos)
#         self.top_sub   = self.create_subscription(Image, self.top_topic,   self.top_cb,   qos)

#         # ---------- Timer to stop after duration ----------
#         self.start_time = self.get_clock().now()
#         self.stop_timer = self.create_timer(0.5, self._check_stop)

#         self.get_logger().info(f"Recording started for {self.duration} s")
#         self.get_logger().info(f" Front camera: {self.front_topic} -> {self.front_out}")
#         self.get_logger().info(f" Top camera:   {self.top_topic}   -> {self.top_out}")
#         self.get_logger().info(f" FourCC: {self.fourcc_str}, FPS metadata: {self.fps}")

#     # ---------- Callbacks ----------
#     def front_cb(self, msg: Image):
#         self._write_frame(msg, which='front')

#     def top_cb(self, msg: Image):
#         self._write_frame(msg, which='top')

#     def _write_frame(self, msg: Image, which: str):
#         try:
#             cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
#         except Exception as e:
#             self.get_logger().warn(f"cv_bridge conversion failed ({which}): {e}")
#             return

#         h, w = cv_img.shape[:2]
#         fourcc = cv2.VideoWriter_fourcc(*self.fourcc_str)

#         if which == 'front':
#             with self.front_lock:
#                 if self.front_writer is None:
#                     self.front_writer = cv2.VideoWriter(self.front_out, fourcc, self.fps, (w, h), True)
#                     if not self.front_writer.isOpened():
#                         self.get_logger().error("Failed to open front VideoWriter. "
#                                                 "Try a different fourcc (e.g., 'avc1') or ensure codecs are installed.")
#                         return
#                 self.front_writer.write(cv_img)

#         elif which == 'top':
#             with self.top_lock:
#                 if self.top_writer is None:
#                     self.top_writer = cv2.VideoWriter(self.top_out, fourcc, self.fps, (w, h), True)
#                     if not self.top_writer.isOpened():
#                         self.get_logger().error("Failed to open top VideoWriter. "
#                                                 "Try a different fourcc (e.g., 'avc1') or ensure codecs are installed.")
#                         return
#                 self.top_writer.write(cv_img)

#     # ---------- Stop logic ----------
#     def _check_stop(self):
#         elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
#         if elapsed >= self.duration:
#             self.get_logger().info("Time reached. Stopping and finalizing videos...")
#             self._finalize_and_shutdown()

#     def _finalize_and_shutdown(self):
#         try:
#             with self.front_lock:
#                 if self.front_writer is not None:
#                     self.front_writer.release()
#                     self.front_writer = None
#             with self.top_lock:
#                 if self.top_writer is not None:
#                     self.top_writer.release()
#                     self.top_writer = None
#         finally:
#             # Let ROS2 shut down cleanly
#             self.stop_timer.cancel()
#             self.get_logger().info(f"Saved: {self.front_out}, {self.top_out}")
#             rclpy.shutdown()

# def main():
#     rclpy.init()
#     node = DualRealSenseRecorder()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         node.get_logger().info("Interrupted by user. Finalizing files...")
#         node._finalize_and_shutdown()
#     except Exception as e:
#         node.get_logger().error(f"Unhandled exception: {e}")
#         node._finalize_and_shutdown()

# if __name__ == '__main__':
#     main()

#!/usr/bin/env python3
# Records RGB + Depth from two RealSense cameras to MP4 and per-frame images.
# Depth:
#   - Saves RAW 16-bit PNGs (millimeters) to a folder
#   - Saves 8-bit colormap MP4 visualization
#   - Optionally also saves 8-bit viz PNGs per frame
#
# Parameters (override with --ros-args -p name:=value):
#   front_topic_rgb:   /front/cam2/color/image_raw
#   top_topic_rgb:     /top/cam1/color/image_raw
#   front_topic_depth: /front/cam2/aligned_depth_to_color/image_raw
#   top_topic_depth:   /top/cam1/aligned_depth_to_color/image_raw
#   front_rgb_output:  front_rgb.mp4
#   top_rgb_output:    top_rgb.mp4
#   front_depth_mp4:   front_depth.mp4
#   top_depth_mp4:     top_depth.mp4
#   front_depth_dir:   front_depth_frames
#   top_depth_dir:     top_depth_frames
#   save_depth_viz_png: False
#   fps: 30.0
#   duration_sec: 120
#   fourcc: mp4v
#   depth_max_m: 4.0    # distance mapped to 255 in visualization (meters)

import os
import cv2
import rclpy
import threading
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class DualRealSenseRecorder(Node):
    def __init__(self):
        super().__init__('dual_realsense_recorder')

        # ---------- Parameters ----------
        # RGB topics/outputs
        self.declare_parameter('front_topic_rgb', '/front/cam2/color/image_raw')
        self.declare_parameter('top_topic_rgb',   '/top/cam1/color/image_raw')
        self.declare_parameter('front_rgb_output', 'front_rgb.mp4')
        self.declare_parameter('top_rgb_output',   'top_rgb.mp4')

        # Depth topics/outputs
        self.declare_parameter('front_topic_depth', '/front/cam2/aligned_depth_to_color/image_raw')
        self.declare_parameter('top_topic_depth',   '/top/cam1/aligned_depth_to_color/image_raw')
        self.declare_parameter('front_depth_mp4', 'front_depth.mp4')
        self.declare_parameter('top_depth_mp4',   'top_depth.mp4')
        self.declare_parameter('front_depth_dir', 'front_depth_frames')
        self.declare_parameter('top_depth_dir',   'top_depth_frames')
        self.declare_parameter('save_depth_viz_png', False)

        # Shared settings
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('duration_sec', 60)
        self.declare_parameter('fourcc', 'mp4v')
        self.declare_parameter('depth_max_m', 4.0)

        # ---------- Read parameters ----------
        self.front_topic_rgb = self.get_parameter('front_topic_rgb').get_parameter_value().string_value
        self.top_topic_rgb   = self.get_parameter('top_topic_rgb').get_parameter_value().string_value
        self.front_rgb_out   = self.get_parameter('front_rgb_output').get_parameter_value().string_value
        self.top_rgb_out     = self.get_parameter('top_rgb_output').get_parameter_value().string_value

        self.front_topic_depth = self.get_parameter('front_topic_depth').get_parameter_value().string_value
        self.top_topic_depth   = self.get_parameter('top_topic_depth').get_parameter_value().string_value
        self.front_depth_mp4   = self.get_parameter('front_depth_mp4').get_parameter_value().string_value
        self.top_depth_mp4     = self.get_parameter('top_depth_mp4').get_parameter_value().string_value
        self.front_depth_dir   = self.get_parameter('front_depth_dir').get_parameter_value().string_value
        self.top_depth_dir     = self.get_parameter('top_depth_dir').get_parameter_value().string_value
        self.save_depth_viz_png = self.get_parameter('save_depth_viz_png').get_parameter_value().bool_value

        self.fps        = float(self.get_parameter('fps').get_parameter_value().double_value)
        self.duration   = int(self.get_parameter('duration_sec').get_parameter_value().integer_value)
        self.fourcc_str = self.get_parameter('fourcc').get_parameter_value().string_value
        self.depth_max_m = float(self.get_parameter('depth_max_m').get_parameter_value().double_value)

        # ---------- Setup ----------
        self.bridge = CvBridge()
        self.fourcc = cv2.VideoWriter_fourcc(*self.fourcc_str)

        # RGB writers/locks
        self.front_rgb_writer = None
        self.top_rgb_writer   = None
        self.front_rgb_lock = threading.Lock()
        self.top_rgb_lock   = threading.Lock()

        # Depth writers/locks + frame counters
        self.front_depth_writer = None
        self.top_depth_writer   = None
        self.front_depth_lock = threading.Lock()
        self.top_depth_lock   = threading.Lock()
        self.front_depth_count = 0
        self.top_depth_count   = 0

        # Ensure depth directories exist
        os.makedirs(self.front_depth_dir, exist_ok=True)
        os.makedirs(self.top_depth_dir, exist_ok=True)

        # QoS: BEST_EFFORT is common for camera streams
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )

        # ---------- Subscriptions ----------
        self.front_rgb_sub = self.create_subscription(Image, self.front_topic_rgb, self.front_rgb_cb, qos)
        self.top_rgb_sub   = self.create_subscription(Image, self.top_topic_rgb,   self.top_rgb_cb,   qos)
        self.front_depth_sub = self.create_subscription(Image, self.front_topic_depth, self.front_depth_cb, qos)
        self.top_depth_sub   = self.create_subscription(Image, self.top_topic_depth,   self.top_depth_cb,   qos)

        # ---------- Timer to stop after duration ----------
        self.start_time = self.get_clock().now()
        self.stop_timer = self.create_timer(0.5, self._check_stop)

        # ---------- Logs ----------
        self.get_logger().info(f"Recording {self.duration}s @ {self.fps} FPS (metadata), FourCC={self.fourcc_str}")
        self.get_logger().info(f" RGB Front: {self.front_topic_rgb} -> {self.front_rgb_out}")
        self.get_logger().info(f" RGB Top:   {self.top_topic_rgb}   -> {self.top_rgb_out}")
        self.get_logger().info(f" Depth Front: {self.front_topic_depth} -> {self.front_depth_mp4}, frames -> {self.front_depth_dir}")
        self.get_logger().info(f" Depth Top:   {self.top_topic_depth}   -> {self.top_depth_mp4},   frames -> {self.top_depth_dir}")
        self.get_logger().info(f" Depth vis range: 0..{self.depth_max_m:.2f} m; save viz PNGs: {self.save_depth_viz_png}")

    # ---------- RGB Callbacks ----------
    def front_rgb_cb(self, msg: Image):
        self._write_rgb(msg, which='front')

    def top_rgb_cb(self, msg: Image):
        self._write_rgb(msg, which='top')

    def _write_rgb(self, msg: Image, which: str):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f"cv_bridge RGB conversion failed ({which}): {e}")
            return

        h, w = cv_img.shape[:2]
        if which == 'front':
            with self.front_rgb_lock:
                if self.front_rgb_writer is None:
                    self.front_rgb_writer = cv2.VideoWriter(self.front_rgb_out, self.fourcc, self.fps, (w, h), True)
                    if not self.front_rgb_writer.isOpened():
                        self.get_logger().error("Failed to open front RGB VideoWriter. Try different 'fourcc' or install codecs.")
                        return
                self.front_rgb_writer.write(cv_img)
        else:
            with self.top_rgb_lock:
                if self.top_rgb_writer is None:
                    self.top_rgb_writer = cv2.VideoWriter(self.top_rgb_out, self.fourcc, self.fps, (w, h), True)
                    if not self.top_rgb_writer.isOpened():
                        self.get_logger().error("Failed to open top RGB VideoWriter. Try different 'fourcc' or install codecs.")
                        return
                self.top_rgb_writer.write(cv_img)

    # ---------- Depth Callbacks ----------
    def front_depth_cb(self, msg: Image):
        self._write_depth(msg, which='front')

    def top_depth_cb(self, msg: Image):
        self._write_depth(msg, which='top')

    def _write_depth(self, msg: Image, which: str):
        """
        Converts incoming depth to:
          - RAW 16-bit PNG in millimeters
          - 8-bit viz frame (colormap) for MP4 and optional PNG
        Supports 16UC1 (mm) and 32FC1 (meters).
        """
        try:
            # Keep native encoding to access original dtype
            cv_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().warn(f"cv_bridge depth conversion failed ({which}): {e}")
            return

        # Convert to uint16 millimeters for RAW saving
        if cv_depth.dtype == 'uint16':
            depth_mm = cv_depth.copy()
        elif cv_depth.dtype == 'float32':
            # Assume meters -> convert to millimeters, clamp negatives to 0
            depth_mm = (cv_depth * 1000.0).clip(min=0).astype('uint16')
        else:
            self.get_logger().warn(f"Unsupported depth dtype {cv_depth.dtype} ({which}); expected uint16 or float32.")
            return

        h, w = depth_mm.shape[:2]

        # Save RAW 16-bit PNG
        count, out_dir = (self.front_depth_count, self.front_depth_dir) if which == 'front' else (self.top_depth_count, self.top_depth_dir)
        # Use ROS timestamp (nanoseconds) for deterministic filenames
        stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        raw_name = os.path.join(out_dir, f"{stamp_ns}_raw16.png")
        try:
            cv2.imwrite(raw_name, depth_mm)  # 16-bit PNG
        except Exception as e:
            self.get_logger().warn(f"Failed to write depth RAW PNG ({which}): {e}")

        # Build visualization (8-bit + colormap) for MP4 and optional PNGs
        viz = self._depth_to_viz(depth_mm, self.depth_max_m)
        viz_bgr = cv2.applyColorMap(viz, cv2.COLORMAP_TURBO)  # BGR

        if self.save_depth_viz_png:
            viz_name = os.path.join(out_dir, f"{stamp_ns}_viz8.png")
            try:
                cv2.imwrite(viz_name, viz_bgr)
            except Exception as e:
                self.get_logger().warn(f"Failed to write depth VIZ PNG ({which}): {e}")

        # Write to MP4 (create writer lazily)
        if which == 'front':
            with self.front_depth_lock:
                if self.front_depth_writer is None:
                    self.front_depth_writer = cv2.VideoWriter(self.front_depth_mp4, self.fourcc, self.fps, (w, h), True)
                    if not self.front_depth_writer.isOpened():
                        self.get_logger().error("Failed to open front depth VideoWriter. Try different 'fourcc' or install codecs.")
                        return
                self.front_depth_writer.write(viz_bgr)
                self.front_depth_count += 1
        else:
            with self.top_depth_lock:
                if self.top_depth_writer is None:
                    self.top_depth_writer = cv2.VideoWriter(self.top_depth_mp4, self.fourcc, self.fps, (w, h), True)
                    if not self.top_depth_writer.isOpened():
                        self.get_logger().error("Failed to open top depth VideoWriter. Try different 'fourcc' or install codecs.")
                        return
                self.top_depth_writer.write(viz_bgr)
                self.top_depth_count += 1

    def _depth_to_viz(self, depth_mm: 'cv2.Mat', depth_max_m: float):
        """
        Map depth in mm to 0..255 (uint8) using a fixed max range (depth_max_m).
        0 or invalid depths remain 0 (black); nearer = lower value, farther = higher value.
        """
        max_mm = max(1, int(depth_max_m * 1000.0))
        # Clip and scale
        depth_clipped = depth_mm.copy()
        depth_clipped[depth_clipped > max_mm] = max_mm
        # Normalize to 0..255: depth / max_mm * 255
        viz = (depth_clipped.astype('float32') * (255.0 / float(max_mm))).astype('uint8')
        return viz

    # ---------- Stop logic ----------
    def _check_stop(self):
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed >= self.duration:
            self.get_logger().info("Time reached. Stopping and finalizing videos...")
            self._finalize_and_shutdown()

    def _finalize_and_shutdown(self):
        try:
            with self.front_rgb_lock:
                if self.front_rgb_writer is not None:
                    self.front_rgb_writer.release()
                    self.front_rgb_writer = None
            with self.top_rgb_lock:
                if self.top_rgb_writer is not None:
                    self.top_rgb_writer.release()
                    self.top_rgb_writer = None
            with self.front_depth_lock:
                if self.front_depth_writer is not None:
                    self.front_depth_writer.release()
                    self.front_depth_writer = None
            with self.top_depth_lock:
                if self.top_depth_writer is not None:
                    self.top_depth_writer.release()
                    self.top_depth_writer = None
        finally:
            try:
                self.stop_timer.cancel()
            except Exception:
                pass
            self.get_logger().info(
                "Saved videos:\n"
                f"  RGB:   {self.front_rgb_out}, {self.top_rgb_out}\n"
                f"  Depth: {self.front_depth_mp4}, {self.top_depth_mp4}\n"
                "Depth frames saved to:\n"
                f"  {self.front_depth_dir}\n"
                f"  {self.top_depth_dir}"
            )
            rclpy.shutdown()

def main():
    rclpy.init()
    node = DualRealSenseRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user. Finalizing files...")
        node._finalize_and_shutdown()
    except Exception as e:
        node.get_logger().error(f"Unhandled exception: {e}")
        node._finalize_and_shutdown()

if __name__ == '__main__':
    main()