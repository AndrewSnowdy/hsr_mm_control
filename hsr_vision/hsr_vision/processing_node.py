import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration

from sensor_msgs.msg import Image, LaserScan
from vision_msgs.msg import Detection2DArray
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PointStamped

from cv_bridge import CvBridge
import tf2_ros
from tf2_geometry_msgs import do_transform_point


class HSRPersonTracker(Node):
    def __init__(self):
        super().__init__("hsr_person_tracker")

        # --- Params (keep them hard-coded for simplicity) ---
        self.depth_topic = "/rgb_PS1080_PrimeSense/depth_registered/image_raw"
        self.det_topic = "/detected_objects"
        self.scan_topic = "/scan"
        self.base_frame = "odom"

        # --- Class-Specific Configs ---
        self.class_configs = {
            "person":      {"thresh": 2,  "dist": 1.2, "timeout": 0.5, "color": (0.2, 0.4, 1.0)}, # Blue
            "prox_button": {"thresh": 12, "dist": 0.5, "timeout": 10.0, "color": (1.0, 0.0, 0.0)}, # Red
            "push_button": {"thresh": 12, "dist": 0.5, "timeout": 10.0, "color": (0.0, 1.0, 0.0)}, # Green
            "door":        {"thresh": 10,  "dist": 0.5, "timeout": 15.0,  "color": (1.0, 0.5, 0.0)}, # orange
            "default":     {"thresh": 5,  "dist": 0.5, "timeout": 1.0, "color": (0.5, 0.5, 0.5)}
}

        # Intrinsics for the depth_registered image
        self.fx = 525.0 
        self.fy = 525.0
        self.cx = 320.0
        self.cy = 240.0

        # Association + smoothing
        self.alpha_pos = 0.3
        self.alpha_vel = 0.3 
        # self.max_age_s = 1.0

        # Visualization
        self.pred_steps = 4
        self.pred_dt = 0.2
        self.marker_topic = "/visualization_marker"

        # --- State ---
        self.bridge = CvBridge()
        self.latest_depth = None
        self.latest_depth_is_mm = False

        self.tracks = {}  # id -> dict(pos=[x,y], vel=[vx,vy], last_t=float)
        self.next_id = 0

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Pub/Sub
        self.create_subscription(Image, self.depth_topic, self.depth_cb, 10)
        self.create_subscription(Detection2DArray, self.det_topic, self.det_cb, 10)
        self.create_subscription(LaserScan, self.scan_topic, self.scan_cb, 10)
        self.marker_pub = self.create_publisher(MarkerArray, self.marker_topic, 10)

        self.get_logger().info("HSRPersonTracker up. Tracking buttons and people!")
        # Create a diagnostic logger that runs every 2.0 seconds
        self.create_timer(2.0, self._log_diagnostics)

    def scan_cb(self, msg: LaserScan):
        self.latest_scan = msg

    def depth_cb(self, msg: Image) -> None:
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        self.latest_depth_is_mm = (cv_img.dtype == np.uint16)

        # Store as float meters
        if self.latest_depth_is_mm:
            self.latest_depth = cv_img.astype(np.float32) / 1000.0
        else:
            self.latest_depth = cv_img.astype(np.float32)

    def det_cb(self, msg: Detection2DArray) -> None:
        if self.latest_depth is None: return
        cam_frame = msg.header.frame_id
        now = self.get_clock().now().nanoseconds * 1e-9

        # --- STEP 1: Project all detections into 3D base_link ---
        observations = []
        for det in msg.detections:
            label = det.results[0].hypothesis.class_id.lower()
            if label not in self.class_configs: continue
            # if label == "door":
            #     # 1. Get 2D Edges from the Bounding Box
            #     yaw_offset = -0.13
            #     pixel_inset = 5

            #     # 2. Calculate coordinates moved inward from the edges
            #     u_l_safe = int(det.bbox.center.position.x - det.bbox.size_x / 2.0) + pixel_inset
            #     u_r_safe = int(det.bbox.center.position.x + det.bbox.size_x / 2.0) - pixel_inset
            #     v_bottom = int(det.bbox.center.position.y + det.bbox.size_y / 2.0)

            #     # 2. Get Depth from Depth Map
            #     z_l = self._stable_depth_at(u_l_safe, v_bottom, win=6)
            #     z_r = self._stable_depth_at(u_r_safe, v_bottom, win=6)
                
            #     x_l_cam = (u_l_safe - self.cx) / self.fx + yaw_offset
            #     y_l_cam = (v_bottom - self.cy) / self.fy
                
            #     # Project Ray R with offset
            #     x_r_cam = (u_r_safe - self.cx) / self.fx + yaw_offset
            #     y_r_cam = (v_bottom - self.cy) / self.fy

            #     # Use a depth of 1.0 to create the 3D point in Camera Space
            #     # We use the optical frame convention (Z-Forward, X-Right, Y-Down)
            #     if z_l is None:
            #         z_l = self._get_lidar_dist_at_angle(ang_l)
            #         if z_l == 5.0:
            #             self.get_logger().warn(f"Door LEFT edge: No Depth or Lidar hit. Defaulting to 5.0m")
            #     if z_r is None:
            #         z_r = self._get_lidar_dist_at_angle(ang_r)
            #         if z_r == 5.0:
            #             self.get_logger().warn(f"Door RIGHT edge: No Depth or Lidar hit. Defaulting to 5.0m")

            #     p_l_cam = [x_l_cam * (z_l or temp_z), y_l_cam * (z_l or temp_z), (z_l or temp_z)]
            #     p_r_cam = [x_r_cam * (z_r or temp_z), y_r_cam * (z_r or temp_z), (z_r or temp_z)]

            #     # --- THE TF STEP: Transform from 'head_rgbd_sensor_link' to 'odom' ---
            #     # This step automatically accounts for the head's pan/tilt rotation!
            #     p_l_vis = self._transform_point(p_l_cam, cam_frame, self.base_frame)
            #     p_r_vis = self._transform_point(p_r_cam, cam_frame, self.base_frame)
            if label == "door":
                yaw_offset = -0.125
                u_l = int(det.bbox.center.position.x - det.bbox.size_x / 2.0) + 5
                u_r = int(det.bbox.center.position.x + det.bbox.size_x / 2.0) - 5
                v_bottom = int(det.bbox.center.position.y + det.bbox.size_y / 2.0)

                # Calculate angles directly from pixels (no camera depth used here)
                ang_l = math.atan2((u_l - self.cx), self.fx) + yaw_offset
                ang_r = math.atan2((u_r - self.cx), self.fx) + yaw_offset

                # Get the REAL distance from the Lidar at those angles
                z_l_real = self._get_lidar_dist_at_angle(ang_l)
                z_r_real = self._get_lidar_dist_at_angle(ang_r)

                if z_l_real == 5.0 or z_r_real == 5.0:
                    self.get_logger().warn("Lidar missed the door frame; using default distance.")

                # Project into 3D using Lidar distance
                p_l_cam = [math.sin(ang_l) * z_l_real, ((v_bottom - self.cy) / self.fy) * z_l_real, math.cos(ang_l) * z_l_real]
                p_r_cam = [math.sin(ang_r) * z_r_real, ((v_bottom - self.cy) / self.fy) * z_r_real, math.cos(ang_r) * z_r_real]

                # Transform and Refine as usual
                p_l_vis = self._transform_point(p_l_cam, cam_frame, self.base_frame)
                p_r_vis = self._transform_point(p_r_cam, cam_frame, self.base_frame)

                if p_l_vis and p_r_vis:
                    # --- NEW REFINEMENT STEP ---   
                    # Use your new scan helper to "snap" these points to the physical wall edges
                    p_l, p_r = self._refine_door_with_scan(p_l_vis, p_r_vis)
                    
                    # 4. Use the REFINED points for the final observation
                    mid_x = (p_l[0] + p_r[0]) / 2.0
                    mid_y = (p_l[1] + p_r[1]) / 2.0
                    mid_z = (p_l[2] + p_r[2]) / 2.0
                    
                    observations.append({
                        'pos': (mid_x, mid_y, mid_z), 
                        'label': label,
                        'p_l': p_l, 'p_r': p_r, 
                        'bbox': det.bbox, 'cam_frame': cam_frame
                    })
            else:
                yaw_offset = -0.13
                # Standard logic for people/buttons
                u, v = int(det.bbox.center.position.x), int(det.bbox.center.position.y)
                z = self._stable_depth_at(u, v)
                if z is None: continue

                yaw_offset = -0.1 

                # Project to 3D with the calibration offset
                # We apply the offset to the horizontal (X) component
                x_cam = ((u - self.cx) / self.fx + yaw_offset) * z
                y_cam = ((v - self.cy) / self.fy) * z
                z_cam = z

                # Transform using the head's current TF
                p_base = self._transform_point((x_cam, y_cam, z_cam), cam_frame, self.base_frame)

                if p_base:
                    observations.append({'pos': p_base, 'label': label}) 

        # --- STEP 2: Match to Tracks (Your previous logic) ---
        used_obs_indices = set()
        for tid, tr in self.tracks.items():
            config = self.class_configs.get(tr['label'], self.class_configs["default"])
            best_j= None
            best_d = config["dist"]

            for j, obs in enumerate(observations):
                if j in used_obs_indices or obs['label'] != tr['label']: continue
                d = math.hypot(obs['pos'][0] - tr['pos'][0], obs['pos'][1] - tr['pos'][1])
                if d < best_d:
                    best_d = d
                    best_j = j

            if best_j is not None:
                used_obs_indices.add(best_j)
                obs = observations[best_j]
                self._update_track(tid, obs['pos'][0], obs['pos'][1], obs['pos'][2], now, 
                                bbox=obs.get('bbox'), cam_frame=obs.get('cam_frame'), 
                                p_l=obs.get('p_l'), p_r=obs.get('p_r'))

        # --- STEP 3: New Tracks (With Proximity Filter) ---
        for j, obs in enumerate(observations):
            if j in used_obs_indices: 
                continue

            # Check if this "new" detection is too close to ANY existing track
            # (regardless of whether that track was updated this frame)
            is_duplicate = False
            for tr in self.tracks.values():
                # Use a small "minimum birth distance" (e.g., 0.5 meters)
                # If it's closer than this to an existing track, it's likely a glitch
                dist_to_existing = math.hypot(obs['pos'][0] - tr['pos'][0], 
                                              obs['pos'][1] - tr['pos'][1])
                
                if obs['label'] == tr['label'] and dist_to_existing < 0.5:
                    is_duplicate = True
                    break
            
            if not is_duplicate:
                obs = observations[j]
                self._create_track(obs['pos'][0], obs['pos'][1], obs['pos'][2], now, obs['label'],
                                bbox=obs.get('bbox'), cam_frame=obs.get('cam_frame'), 
                                p_l=obs.get('p_l'), p_r=obs.get('p_r'))

        self._prune_tracks(now)
        self._publish_markers()

    # --------------------
    # Tracking
    # --------------------
    def _create_track(self, x: float, y: float, z: float, t: float, label: str, 
                      bbox=None, cam_frame=None, p_l=None, p_r=None) -> None:
        tid = self.next_id
        self.next_id += 1
        
        self.tracks[tid] = {
            "pos": [x, y, z],
            "vel": [0.0, 0.0],
            "last_t": t,
            "hits": 1,
            "label": label,
            "last_bbox": bbox,
            "cam_frame": cam_frame,
            "p_l": p_l,  # Grouped Pillar Left
            "p_r": p_r   # Grouped Pillar Right
        }
        # self.get_logger().info(f"Created {label} track {tid} at [{x:.2f}, {y:.2f}]")

    def _update_track(self, tid: int, x: float, y: float, z: float, t: float, 
                      bbox=None, cam_frame=None, p_l=None, p_r=None) -> None:
        tr = self.tracks[tid]
        dt = max(1e-3, t - tr["last_t"])
        px, py, pz = list(tr["pos"])
        # 1. Smooth the center position
        tr["pos"][0] = self.alpha_pos * x + (1.0 - self.alpha_pos) * tr["pos"][0]
        tr["pos"][1] = self.alpha_pos * y + (1.0 - self.alpha_pos) * tr["pos"][1]
        tr["pos"][2] = self.alpha_pos * z + (1.0 - self.alpha_pos) * tr["pos"][2]

        # 2. Smooth the pillars (Door specific)
        if tr["label"] == "door" and p_l and p_r:
            tr["vel"] = [0.0, 0.0]

            if isinstance(tr.get("p_l"), tuple):
                tr["p_l"] = list(tr["p_l"])
            if isinstance(tr.get("p_r"), tuple):
                tr["p_r"] = list(tr["p_r"])

            if tr.get("p_l") is None: tr["p_l"] = list(p_l)
            if tr.get("p_r") is None: tr["p_r"] = list(p_r)

            # We smooth these so the door doesn't jump if a single depth pixel is noisy
            for i in range(3):
                tr["p_l"][i] = self.alpha_pos * p_l[i] + (1.0 - self.alpha_pos) * tr["p_l"][i]
                tr["p_r"][i] = self.alpha_pos * p_r[i] + (1.0 - self.alpha_pos) * tr["p_r"][i]
            
            tr["last_bbox"] = bbox
            tr["cam_frame"] = cam_frame

        if tr["label"] == "person":
            nvx = (tr["pos"][0] - px) / dt
            nvy = (tr["pos"][1] - py) / dt
            
            tr["vel"][0] = self.alpha_vel * nvx + (1.0 - self.alpha_vel) * tr["vel"][0]
            tr["vel"][1] = self.alpha_vel * nvy + (1.0 - self.alpha_vel) * tr["vel"][1]
            
            if math.hypot(tr["vel"][0], tr["vel"][1]) < 0.05:
                tr["vel"] = [0.0, 0.0]


        tr["hits"] += 1
        tr["last_t"] = t


    def _prune_tracks(self, now: float) -> None:
        to_del = []
        for tid, tr in self.tracks.items():
            # 1. Look up the specific timeout for this object type
            config = self.class_configs.get(tr['label'], self.class_configs["default"])
            max_age = config.get("timeout", 1.0) 

            # 2. Check if the "Grace Period" has expired
            if (now - tr["last_t"]) > max_age:
                to_del.append(tid)

        for tid in to_del:
            del self.tracks[tid]

    # --------------------
    # Depth + TF helpers
    # --------------------
    def _stable_depth_at(self, u: int, v: int, win: int = 2) -> float | None:
        h, w = self.latest_depth.shape[:2]
        u0, u1 = max(0, u - win), min(w, u + win + 1)
        v0, v1 = max(0, v - win), min(h, v + win + 1)
        patch = self.latest_depth[v0:v1, u0:u1]
        valid = patch[np.isfinite(patch) & (patch > 0.1)]
        if valid.size == 0:
            return None
        return float(np.median(valid))

    def _camera_point_to_base_link(self, x: float, y: float, z: float, cam_frame: str):
        try:
            p = PointStamped()
            p.header.frame_id = cam_frame
            p.header.stamp = self.get_clock().now().to_msg()  # consistent with latest TF
            p.point.x, p.point.y, p.point.z = float(x), float(y), float(z)

            tf = self.tf_buffer.lookup_transform(
                self.base_frame,
                cam_frame,
                Time(),  # latest TF for robustness
                timeout=Duration(seconds=0.3),
            )
            out = do_transform_point(p, tf)
            return (out.point.x, out.point.y, out.point.z)
        except Exception as e:
            self.get_logger().warn(f"TF fail {self.base_frame}<-{cam_frame}: {type(e).__name__}: {e}")
            return None

    def _transform_point(self, pt, from_frame, to_frame):
        try:
            p = PointStamped()
            p.header.frame_id = from_frame
            # Using Time() here tells ROS to find the newest transform available
            p.header.stamp = Time().to_msg() 
            p.point.x, p.point.y, p.point.z = float(pt[0]), float(pt[1]), float(pt[2])

            # We ask for Time() (latest) and add a small timeout to let the buffer fill
            tf = self.tf_buffer.lookup_transform(
                to_frame, 
                from_frame, 
                Time(), 
                timeout=Duration(seconds=0.3)
            )
            out = do_transform_point(p, tf)
            return (out.point.x, out.point.y, out.point.z)
        except Exception:
            return None

    def _refine_door_with_scan(self, vision_p_l, vision_p_r):
        if not hasattr(self, 'latest_scan') or self.latest_scan is None:
            return vision_p_l, vision_p_r

        scan = self.latest_scan
        # Use the specific frame from the scan message
        laser_frame = scan.header.frame_id 
        
        # Transform Vision (Odom) -> Laser Frame
        p_l_laser = self._transform_point(vision_p_l, self.base_frame, laser_frame)
        p_r_laser = self._transform_point(vision_p_r, self.base_frame, laser_frame)
        
        if not p_l_laser or not p_r_laser: return vision_p_l, vision_p_r

        # Inset Math (unchanged, this part is solid)
        dx, dy = p_r_laser[0] - p_l_laser[0], p_r_laser[1] - p_l_laser[1]
        dist_total = math.hypot(dx, dy)
        ux, uy = dx / dist_total, dy / dist_total
        
        inset = 0.05
        p_l_in = [p_l_laser[0] + ux * inset, p_l_laser[1] + uy * inset]
        p_r_in = [p_r_laser[0] - ux * inset, p_r_laser[1] - uy * inset]

        def get_stable_dist(target_p):
            angle = math.atan2(target_p[1], target_p[0])
            idx = int(round((angle - scan.angle_min) / scan.angle_increment))

            window_size = 2
            start = max(0, idx - window_size)
            end = min(len(scan.ranges), idx + window_size + 1)

            pts = []
            for k in range(start, end):
                r = scan.ranges[k]
                if 0.1 < r < 30.0 and np.isfinite(r):
                    a = scan.angle_min + k * scan.angle_increment
                    pts.append([r * math.cos(a), r * math.sin(a)])

            if not pts:
                return None

            pts = np.array(pts, dtype=np.float32)
            x = float(np.median(pts[:, 0]))
            y = float(np.median(pts[:, 1]))
            return [x, y, 0.0]

        refined_l = get_stable_dist(p_l_in) or p_l_laser
        refined_r = get_stable_dist(p_r_in) or p_r_laser

        # Transform back to Odom
        return (self._transform_point(refined_l, laser_frame, self.base_frame),
                self._transform_point(refined_r, laser_frame, self.base_frame))

    def _get_lidar_dist_at_angle(self, target_angle_rad):
        if not hasattr(self, 'latest_scan') or self.latest_scan is None:
            return 5.0

        scan = self.latest_scan
        # Calculate the index in the lidar array
        idx = int((target_angle_rad - scan.angle_min) / scan.angle_increment)
        
        # Search a 10-beam window (approx 3 degrees) for the closest solid hit
        window = 5
        start, end = max(0, idx - window), min(len(scan.ranges), idx + window + 1)
        valid = [r for r in scan.ranges[start:end] if 0.1 < r < 30.0]
        
        if not valid:
            return 5.0
        return float(np.median(valid))
    # --------------------
    # Visualization
    # --------------------
    def _publish_markers(self) -> None:
        now_msg = self.get_clock().now().to_msg()
        full_array = MarkerArray()

        # 1. Add a single "Clear All" marker at the start of the array
        clear_marker = Marker()
        clear_marker.action = Marker.DELETEALL
        full_array.markers.append(clear_marker)

        self._add_people_to_array(full_array, now_msg)
        self._add_buttons_to_array(full_array, now_msg)
        self._add_doors_to_array(full_array, now_msg)

        # 4. Publish EVERYTHING once
        if len(full_array.markers) > 1: # Only publish if there's more than just the clear marker
            self.marker_pub.publish(full_array)

    def _add_people_to_array(self, arr: MarkerArray, now_msg) -> None:
        for tid, tr in self.tracks.items():
            if tr["label"] != "person": continue
            config = self.class_configs.get("person")
            if tr["hits"] < config["thresh"]: continue

            px, py, _ = tr["pos"]
            vx, vy = tr.get("vel", [0.0, 0.0])

            for step in range(self.pred_steps):
                m = Marker()
                m.header.frame_id, m.header.stamp = self.base_frame, now_msg
                m.ns, m.id = f"person_{tid}", step
                m.type, m.action = Marker.CYLINDER, Marker.ADD

                t_offset = step * self.pred_dt
                m.pose.position.x = float(px + vx * t_offset)
                m.pose.position.y = float(py + vy * t_offset)
                m.pose.position.z, m.pose.orientation.w = 0.01, 1.0

                scale_factor = 1.0 - (step / (self.pred_steps + 1))
                radius = 0.35 * scale_factor
                m.scale.x = m.scale.y = radius * 2.0
                m.scale.z = 0.02

                m.color.r, m.color.g, m.color.b = config["color"]
                m.color.a = float(max(0.1, 0.7 - 0.1 * step))
                arr.markers.append(m)

    def _add_buttons_to_array(self, arr: MarkerArray, now_msg) -> None:
        for tid, tr in self.tracks.items():
            if "button" not in tr["label"]: continue
            config = self.class_configs.get(tr["label"], self.class_configs["default"])
            if tr["hits"] < config["thresh"]: continue

            m = Marker()
            m.header.frame_id, m.header.stamp = self.base_frame, now_msg
            m.ns, m.id = f"button_{tid}", 0
            m.type, m.action = Marker.SPHERE, Marker.ADD

            m.pose.position.x, m.pose.position.y, m.pose.position.z = tr["pos"]
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.12
            
            m.color.r, m.color.g, m.color.b = config["color"]
            m.color.a = 0.8
            arr.markers.append(m)

    def _add_doors_to_array(self, arr: MarkerArray, now_msg) -> None:
        for tid, tr in self.tracks.items():
            if tr["label"] != "door" or tr["hits"] < self.class_configs["door"]["thresh"]:
                continue

            p_l, p_r = tr.get("p_l"), tr.get("p_r")
            if not p_l or not p_r: continue

            # Pillars
            for i, pt in enumerate([p_l, p_r]):
                m = Marker()
                m.header.frame_id, m.header.stamp = self.base_frame, now_msg
                m.ns, m.id = f"door_{tid}_pillars", i
                m.type = Marker.CYLINDER
                m.pose.position.x, m.pose.position.y, m.pose.position.z = pt[0], pt[1], 1.0
                m.scale.x = m.scale.y = 0.08
                m.scale.z = 2.0
                m.color.r, m.color.a = 1.0, 0.8 # Red
                arr.markers.append(m)

            # Arrow Normal Calculation
            dx, dy = p_r[0] - p_l[0], p_r[1] - p_l[1]
            nx, ny = -dy, dx # Perpendicular vector
            mag = math.hypot(nx, ny)
            if mag > 0: nx /= mag; ny /= mag

            # --- FLIP CHECK ---
            # We want the arrow to point away from the robot. 
            # If the robot is at (0,0) in base_link, we check the dot product.
            # (In map frame, you'd need the robot's current pose, but usually, 
            # for a door you just saw, 'away from camera' is a safe default)
            arrow = Marker()
            arrow.header.frame_id, arrow.header.stamp = self.base_frame, now_msg
            arrow.ns, arrow.id = f"door_{tid}_arrow", 0
            arrow.type, arrow.action = Marker.ARROW, Marker.ADD
            
            # Center of the door
            mx, my = (p_l[0] + p_r[0])/2.0, (p_l[1] + p_r[1])/2.0
            arrow.pose.position.x, arrow.pose.position.y, arrow.pose.position.z = mx, my, 0.1
            
            yaw = math.atan2(ny, nx)
            arrow.pose.orientation.z = math.sin(yaw/2.0)
            arrow.pose.orientation.w = math.cos(yaw/2.0)
            arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.6, 0.1, 0.1
            arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = 1.0, 1.0, 1.0, 1.0
            arr.markers.append(arrow)


    def _log_diagnostics(self) -> None:
        if not self.tracks:
            self.get_logger().info("--- No Active Tracks ---")
            return

        self.get_logger().info("--- Current Active Tracks ---")
        self.get_logger().info(f"{'ID':<4} | {'Label':<12} | {'Hits':<5} | {'Status':<10}")
        self.get_logger().info("-" * 40)

        for tid, tr in sorted(self.tracks.items()):
            label = tr['label']
            hits = tr['hits']
            thresh = self.class_configs.get(label, self.class_configs["default"])["thresh"]
            
            # Check if it's currently being visualized
            status = "VISIBLE" if hits >= thresh else f"WAITING ({thresh-hits} left)"
            
            self.get_logger().info(f"{tid:<4} | {label:<12} | {hits:<5} | {status:<10}")

def main(args=None):
    rclpy.init(args=args)
    node = HSRPersonTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()