#!/usr/bin/env python3
"""
auto_localizer.py (ROS1 Noetic)

Fully controlled AMCL auto-localization helper with strong debug/telemetry.

What it does (high level):
1) Waits for /amcl_pose, /scan and AMCL services.
2) Calls /global_localization (uniform particle init).
3) Repeats:
   - (optional) Safety check with LaserScan min range.
   - Spin robot a bit (angular z).
   - Stop.
   - Call /request_nomotion_update (forces AMCL update when stationary).
   - Evaluate AMCL covariance convergence for N consecutive cycles.
4) Publishes state + debug + metrics continuously.

Important note about cmd_vel on TIAGo:
- You currently have BOTH /twist_mux and /auto_localizer publishing to /mobile_base_controller/cmd_vel.
  That’s a conflict by design.
- Prefer publishing into twist_mux (e.g. /twist_mux/input/auto_localizer) and let twist_mux output cmd_vel.
  This file keeps a configurable param "~cmd_vel_topic". Set it to your twist_mux input if available.
"""

import math
import threading
from dataclasses import dataclass

import rospy
import tf2_ros
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Float64, Int32
from std_srvs.srv import Trigger, TriggerResponse


# ---------------------------- Helpers ----------------------------

def _finite_ranges(ranges):
    out = []
    for r in ranges:
        if r is None:
            continue
        if math.isfinite(r) and r > 0.0:
            out.append(r)
    return out


def _yaw_var_from_cov6(cov6):
    # covariance is 6x6 row-major in PoseWithCovariance
    # yaw is rotation around Z => index (5,5) => 5*6+5 = 35
    return float(cov6[35])


def _x_var_from_cov6(cov6):
    # x variance => (0,0) => 0
    return float(cov6[0])


def _y_var_from_cov6(cov6):
    # y variance => (1,1) => 7
    return float(cov6[7])


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


# ---------------------------- Config ----------------------------

@dataclass
class Config:
    # Topics
    cmd_vel_topic: str = "/mobile_base_controller/cmd_vel"
    amcl_pose_topic: str = "/amcl_pose"
    scan_topic: str = "/scan"

    # Services (provided by AMCL)
    srv_global_localization: str = "/global_localization"
    srv_nomotion_update: str = "/request_nomotion_update"

    # Behavior
    auto_start: bool = True
    dry_run: bool = False

    spin_speed: float = 0.45          # rad/s
    spin_step_sec: float = 0.8        # seconds per spin burst
    max_total_sec: float = 80.0       # overall timeout

    # Convergence thresholds
    # By default these are VARIANCE thresholds (not stddev).
    th_var_x: float = 0.05
    th_var_y: float = 0.05
    th_var_yaw: float = 0.10
    required_stable: int = 8

    # If True: compare sqrt(var) to thresholds instead of var.
    # Use this if you want to pass stddev thresholds.
    use_stddev_thresholds: bool = False

    # Safety
    enable_safety_stop: bool = True
    min_obstacle_dist: float = 0.35

    # TF debug / time sanity
    tf_future_tolerance: float = 0.10  # seconds; warn if TF stamp is in the future beyond this


# ---------------------------- Node ----------------------------

class AutoLocalizer:
    def __init__(self):
        rospy.init_node("auto_localizer", anonymous=False)

        self.cfg = self._load_config()

        # Publishers (public topics as in your rosnode info)
        self.pub_state = rospy.Publisher("~state", String, queue_size=10)
        self.pub_debug = rospy.Publisher("~debug", String, queue_size=10)
        self.pub_var_x = rospy.Publisher("~var_x", Float64, queue_size=10)
        self.pub_var_y = rospy.Publisher("~var_y", Float64, queue_size=10)
        self.pub_var_yaw = rospy.Publisher("~var_yaw", Float64, queue_size=10)
        self.pub_pose_age = rospy.Publisher("~pose_age", Float64, queue_size=10)
        self.pub_min_range = rospy.Publisher("~min_range", Float64, queue_size=10)
        self.pub_stable = rospy.Publisher("~stable_count", Int32, queue_size=10)

        # cmd_vel publisher
        self.pub_cmd = rospy.Publisher(self.cfg.cmd_vel_topic, Twist, queue_size=10)

        # TF
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Subscribers
        self.sub_pose = rospy.Subscriber(self.cfg.amcl_pose_topic, PoseWithCovarianceStamped, self._on_amcl_pose, queue_size=10)
        self.sub_scan = rospy.Subscriber(self.cfg.scan_topic, LaserScan, self._on_scan, queue_size=10)

        # Services
        self.srv_start = rospy.Service("~start", Trigger, self._srv_start)

        # State
        self._lock = threading.Lock()
        self._running = False
        self._thread = None

        self.state = "IDLE"
        self._stable = 0

        # Latest data
        self.last_pose = None
        self.last_pose_stamp = None
        self.var_x = float("inf")
        self.var_y = float("inf")
        self.var_yaw = float("inf")

        self.min_range = float("inf")
        self.last_scan_stamp = None

        # Periodic debug output
        rospy.Timer(rospy.Duration(0.5), self._debug_timer)

        rospy.loginfo("auto_localizer ready. auto_start=%s dry_run=%s cmd_vel_topic=%s",
                      self.cfg.auto_start, self.cfg.dry_run, self.cfg.cmd_vel_topic)

        if self.cfg.auto_start:
            # Auto-start after node is up
            rospy.Timer(rospy.Duration(0.2), lambda _evt: self._trigger_start_once(), oneshot=True)

    # ------------------------ Config ------------------------

    def _load_config(self) -> Config:
        c = Config()
        c.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", c.cmd_vel_topic)
        c.amcl_pose_topic = rospy.get_param("~amcl_pose_topic", c.amcl_pose_topic)
        c.scan_topic = rospy.get_param("~scan_topic", c.scan_topic)

        c.srv_global_localization = rospy.get_param("~srv_global_localization", c.srv_global_localization)
        c.srv_nomotion_update = rospy.get_param("~srv_nomotion_update", c.srv_nomotion_update)

        c.auto_start = bool(rospy.get_param("~auto_start", c.auto_start))
        c.dry_run = bool(rospy.get_param("~dry_run", c.dry_run))

        c.spin_speed = float(rospy.get_param("~spin_speed", c.spin_speed))
        c.spin_step_sec = float(rospy.get_param("~spin_step_sec", c.spin_step_sec))
        c.max_total_sec = float(rospy.get_param("~max_total_sec", c.max_total_sec))

        c.th_var_x = float(rospy.get_param("~th_var_x", c.th_var_x))
        c.th_var_y = float(rospy.get_param("~th_var_y", c.th_var_y))
        c.th_var_yaw = float(rospy.get_param("~th_var_yaw", c.th_var_yaw))
        c.required_stable = int(rospy.get_param("~required_stable", c.required_stable))

        c.use_stddev_thresholds = bool(rospy.get_param("~use_stddev_thresholds", c.use_stddev_thresholds))

        c.enable_safety_stop = bool(rospy.get_param("~enable_safety_stop", c.enable_safety_stop))
        c.min_obstacle_dist = float(rospy.get_param("~min_obstacle_dist", c.min_obstacle_dist))

        c.tf_future_tolerance = float(rospy.get_param("~tf_future_tolerance", c.tf_future_tolerance))
        return c

    # ------------------------ Callbacks ------------------------

    def _on_amcl_pose(self, msg: PoseWithCovarianceStamped):
        with self._lock:
            self.last_pose = msg
            self.last_pose_stamp = msg.header.stamp
            cov = msg.pose.covariance
            self.var_x = _x_var_from_cov6(cov)
            self.var_y = _y_var_from_cov6(cov)
            self.var_yaw = _yaw_var_from_cov6(cov)

    def _on_scan(self, msg: LaserScan):
        rs = _finite_ranges(msg.ranges)
        mn = min(rs) if rs else float("inf")
        with self._lock:
            self.min_range = mn
            self.last_scan_stamp = msg.header.stamp

    # ------------------------ Service ------------------------

    def _srv_start(self, _req):
        ok, reason = self.start()
        return TriggerResponse(success=ok, message=reason)

    def _trigger_start_once(self):
        # Avoid double-start due to timers
        self.start()

    # ------------------------ Debug / Telemetry ------------------------

    def _publish_state(self, s: str):
        self.state = s
        self.pub_state.publish(String(data=s))

    def _debug_timer(self, _evt):
        with self._lock:
            pose = self.last_pose
            vx = self.var_x
            vy = self.var_y
            vth = self.var_yaw
            mn = self.min_range
            stable = self._stable
            pose_stamp = self.last_pose_stamp

        now = rospy.Time.now()
        if pose is None or pose_stamp is None or pose_stamp == rospy.Time(0):
            pose_age = float("inf")
        else:
            pose_age = (now - pose_stamp).to_sec()

        # publish scalar topics continuously
        if math.isfinite(vx): self.pub_var_x.publish(Float64(vx))
        if math.isfinite(vy): self.pub_var_y.publish(Float64(vy))
        if math.isfinite(vth): self.pub_var_yaw.publish(Float64(vth))
        if math.isfinite(pose_age): self.pub_pose_age.publish(Float64(pose_age))
        if math.isfinite(mn): self.pub_min_range.publish(Float64(mn))
        self.pub_stable.publish(Int32(stable))

        # TF sanity (optional, purely debug)
        tf_note = ""
        try:
            # Try to read latest transform map->odom (commonly published by AMCL)
            tr = self.tf_buffer.lookup_transform("map", "odom", rospy.Time(0), rospy.Duration(0.05))
            dt = (now - tr.header.stamp).to_sec()
            if dt < -abs(self.cfg.tf_future_tolerance):
                tf_note = f" TF_FUTURE(map->odom dt={dt:.3f}s)"
        except Exception:
            # no spam; only include if you are running and it matters
            pass

        sx = math.sqrt(max(vx, 0.0)) if math.isfinite(vx) else float("inf")
        sy = math.sqrt(max(vy, 0.0)) if math.isfinite(vy) else float("inf")
        sth = math.sqrt(max(vth, 0.0)) if math.isfinite(vth) else float("inf")

        mode = "stddev" if self.cfg.use_stddev_thresholds else "variance"
        msg = (
            f"state={self.state} stable={stable}/{self.cfg.required_stable} "
            f"age={pose_age:.3f}s min_range={mn:.2f} "
            f"var=({vx:.4f},{vy:.4f},{vth:.4f}) std=({sx:.3f},{sy:.3f},{sth:.3f}) "
            f"threshold_mode={mode}{tf_note}"
        )
        self.pub_debug.publish(String(data=msg))

    # ------------------------ Control ------------------------

    def start(self):
        with self._lock:
            if self._running:
                return True, "already running"
            self._running = True
            self._stable = 0

        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        return True, "auto_localizer triggered"

    def _stop_motion(self):
        if self.cfg.dry_run:
            return
        t = Twist()
        t.linear.x = 0.0
        t.angular.z = 0.0
        self.pub_cmd.publish(t)

    def _spin_motion(self):
        if self.cfg.dry_run:
            return
        t = Twist()
        t.linear.x = 0.0
        t.angular.z = float(self.cfg.spin_speed)
        self.pub_cmd.publish(t)

    def _wait_for(self, cond_fn, timeout_sec, poll_hz=20.0):
        rate = rospy.Rate(poll_hz)
        t0 = rospy.Time.now()
        while not rospy.is_shutdown():
            if cond_fn():
                return True
            if (rospy.Time.now() - t0).to_sec() > timeout_sec:
                return False
            rate.sleep()
        return False

    def _have_pose_and_scan(self):
        with self._lock:
            return (self.last_pose is not None) and (self.last_scan_stamp is not None)

    def _converged(self):
        with self._lock:
            vx, vy, vth = self.var_x, self.var_y, self.var_yaw

        if not (math.isfinite(vx) and math.isfinite(vy) and math.isfinite(vth)):
            return False

        if self.cfg.use_stddev_thresholds:
            sx = math.sqrt(max(vx, 0.0))
            sy = math.sqrt(max(vy, 0.0))
            sth = math.sqrt(max(vth, 0.0))
            return (sx < self.cfg.th_var_x) and (sy < self.cfg.th_var_y) and (sth < self.cfg.th_var_yaw)
        else:
            return (vx < self.cfg.th_var_x) and (vy < self.cfg.th_var_y) and (vth < self.cfg.th_var_yaw)

    def _safety_ok(self):
        if not self.cfg.enable_safety_stop:
            return True
        with self._lock:
            mn = self.min_range
        return (mn >= self.cfg.min_obstacle_dist)

    def _run(self):
        t_start = rospy.Time.now()
        self._publish_state("INIT")

        rospy.loginfo("INIT: waiting for /amcl_pose + /scan...")
        ok = self._wait_for(self._have_pose_and_scan, timeout_sec=10.0)
        if not ok:
            rospy.logerr("INIT: timeout waiting for pose/scan topics")
            self._publish_state("FAILED")
            with self._lock:
                self._running = False
            return

        # Prepare AMCL services
        rospy.loginfo("INIT: waiting for services %s and %s ...",
                      self.cfg.srv_global_localization, self.cfg.srv_nomotion_update)
        try:
            rospy.wait_for_service(self.cfg.srv_global_localization, timeout=10.0)
            rospy.wait_for_service(self.cfg.srv_nomotion_update, timeout=10.0)
        except Exception as e:
            rospy.logerr("INIT: services not available: %s", str(e))
            self._publish_state("FAILED")
            with self._lock:
                self._running = False
            return

        # Call global_localization
        try:
            gl = rospy.ServiceProxy(self.cfg.srv_global_localization, Trigger)
            gl()
            rospy.loginfo("Called %s", self.cfg.srv_global_localization)
        except Exception as e:
            rospy.logerr("Failed calling global_localization: %s", str(e))
            self._publish_state("FAILED")
            with self._lock:
                self._running = False
            return

        # Main loop
        self._publish_state("SPIN_WAIT")
        rate = rospy.Rate(20)

        # Service proxy for nomotion update
        nomo = rospy.ServiceProxy(self.cfg.srv_nomotion_update, Trigger)

        # Remember last pose stamp to detect "fresh" updates after nomotion call
        with self._lock:
            last_pose_stamp_seen = self.last_pose_stamp

        while not rospy.is_shutdown():
            # timeout
            if (rospy.Time.now() - t_start).to_sec() > self.cfg.max_total_sec:
                rospy.logerr("Timeout after %.1fs without convergence", self.cfg.max_total_sec)
                self._stop_motion()
                self._publish_state("FAILED")
                break

            # Safety stop
            if not self._safety_ok():
                self._stop_motion()
                self._publish_state("SAFETY_STOP")
                # wait until safe again
                while not rospy.is_shutdown() and not self._safety_ok():
                    rate.sleep()
                self._publish_state("SPIN_WAIT")

            # Spin burst
            self._publish_state("SPIN")
            spin_end = rospy.Time.now() + rospy.Duration(self.cfg.spin_step_sec)
            while not rospy.is_shutdown() and rospy.Time.now() < spin_end:
                self._spin_motion()
                rate.sleep()

            # Stop and request nomotion update
            self._stop_motion()
            self._publish_state("SPIN_WAIT")

            try:
                nomo()
            except Exception as e:
                rospy.logwarn("request_nomotion_update failed: %s", str(e))

            # Wait for a newer amcl_pose stamp (best-effort)
            def pose_updated():
                with self._lock:
                    if self.last_pose_stamp is None:
                        return False
                    return self.last_pose_stamp != last_pose_stamp_seen

            self._wait_for(pose_updated, timeout_sec=2.0, poll_hz=50.0)

            with self._lock:
                last_pose_stamp_seen = self.last_pose_stamp

            # Check convergence
            if self._converged():
                with self._lock:
                    self._stable += 1
                    stable_now = self._stable
                if stable_now >= self.cfg.required_stable:
                    rospy.loginfo("Converged with stable=%d/%d", stable_now, self.cfg.required_stable)
                    self._stop_motion()
                    self._publish_state("SUCCESS")
                    break
            else:
                with self._lock:
                    self._stable = 0

            rate.sleep()

        with self._lock:
            self._running = False

    # ------------------------ Main ------------------------

    def spin(self):
        rospy.spin()


if __name__ == "__main__":
    node = AutoLocalizer()
    node.spin()
