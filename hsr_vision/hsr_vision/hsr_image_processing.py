import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration

from sensor_msgs.msg import Image
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
        self.base_frame = "odom"

        # --- Class-Specific Configs ---
        self.class_configs = {
            "person":      {"thresh": 2,  "dist": 0.8, "timeout": 0.5, "color": (0.2, 0.4, 1.0)}, # Blue
            "prox_button": {"thresh": 12, "dist": 0.5, "timeout": 10.0, "color": (1.0, 0.0, 0.0)}, # Red
            "push_button": {"thresh": 12, "dist": 0.5, "timeout": 10.0, "color": (0.0, 1.0, 0.0)}, # Green
            "door":        {"thresh": 15,  "dist": 1.0, "timeout": 5.0,  "color": (1.0, 0.5, 0.0)}, # orange
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
        self.marker_pub = self.create_publisher(MarkerArray, self.marker_topic, 10)

        self.get_logger().info("HSRPersonTracker up. Tracking buttons and people!")

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

            if label == "door":
                # 1. Get 2D Edges (Bottom corners are more stable for floor centers)
                u_l = int(det.bbox.center.position.x - det.bbox.size_x / 2.0)
                u_r = int(det.bbox.center.position.x + det.bbox.size_x / 2.0)
                v_bottom = int(det.bbox.center.position.y + det.bbox.size_y / 2.0)

                # 2. Project Edges to 3D
                z_l = self._stable_depth_at(u_l, v_bottom)
                z_r = self._stable_depth_at(u_r, v_bottom)
                
                if z_l is None or z_r is None: continue

                p_l = self._camera_point_to_base_link((u_l-self.cx)*z_l/self.fx, (v_bottom-self.cy)*z_l/self.fy, z_l, cam_frame)
                p_r = self._camera_point_to_base_link((u_r-self.cx)*z_r/self.fx, (v_bottom-self.cy)*z_r/self.fy, z_r, cam_frame)

                if p_l and p_r:
                    # 3. Calculate Floor Midpoint
                    mid_x = (p_l[0] + p_r[0]) / 2.0
                    mid_y = (p_l[1] + p_r[1]) / 2.0
                    mid_z = (p_l[2] + p_r[2]) / 2.0
                    
                    observations.append({
                        'pos': (mid_x, mid_y, mid_z), 
                        'label': label,
                        'p_l': p_l, 'p_r': p_r, # Store raw edges for markers
                        'bbox': det.bbox, 'cam_frame': cam_frame
                    })
            else:
                # Standard logic for people/buttons
                u, v = int(det.bbox.center.position.x), int(det.bbox.center.position.y)
                z = self._stable_depth_at(u, v)
                if z is None: continue
                p_base = self._camera_point_to_base_link((u-self.cx)*z/self.fx, (v-self.cy)*z/self.fy, z, cam_frame)
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
        self.get_logger().info(f"Created {label} track {tid} at [{x:.2f}, {y:.2f}]")

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