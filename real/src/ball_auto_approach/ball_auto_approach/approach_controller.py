import math
from time import monotonic

from ball_approach_interfaces.msg import BallDetection, BallEstimate
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String

from .common import LATEST, run, seconds
from .control_mux import ControlMux

ACTIVE = ('CAMERA_ALIGN', 'DETECTING', 'ALIGNING', 'APPROACHING')


def velocity(x=0., y=0., yaw=0.):
    msg = Twist()
    msg.linear.x, msg.linear.y, msg.angular.z = float(x), float(y), float(yaw)
    return msg


def clearance(estimate, x, y, half_length, half_width, margin):
    # Bound the swept footprint during reaction and stopping. Dimensions
    # are measured from base_link; choose symmetric bounds around it.
    ahead_x, ahead_y = x * .4, y * .4
    x0, x1 = -half_length + min(0., ahead_x), half_length + max(0., ahead_x)
    y0, y1 = -half_width + min(0., ahead_y), half_width + max(0., ahead_y)
    if any(x0 - margin <= p.x <= x1 + margin and y0 - margin <= p.y <= y1 + margin
           for p in estimate.obstacles):
        return False
    # Check the target as well. Its returns are excluded from obstacles,
    # but the sphere itself must stay outside the swept footprint.
    ball = estimate.center
    dx = ball.x - min(max(ball.x, x0), x1)
    dy = ball.y - min(max(ball.y, y0), y1)
    return math.hypot(dx, dy) > estimate.ball_radius + margin


def camera_offset(detection, coefficients):
    """Estimate bearing and depth from the image center and check apparent size."""
    if len(coefficients) != 9:
        return None
    u0, ua, ur, v0, va, vr, d0, da, dr = coefficients
    determinant = ua * vr - ur * va
    if abs(determinant) < 1e-6:
        return None
    u, v = detection.u - u0, detection.v - v0
    theta = (u * vr - ur * v) / determinant
    inverse = (ua * v - u * va) / determinant
    if not math.isfinite(theta) or not math.isfinite(inverse) or inverse <= 0:
        return None
    distance = 1. / inverse
    if (not .30 <= distance <= .53 or abs(theta) > .17
            or not abs(detection.diameter - (d0 + da * theta + dr * inverse)) <= 20.):
        return None
    return theta, distance


class ApproachController(Node):
    def __init__(self):
        super().__init__('approach_controller')
        self.declare_parameters('', [
            ('rate', 20.), ('sensor_timeout', .3), ('joy_timeout', .15),
            ('approach_timeout', 12.), ('detection_timeout', 1.),
            ('target_distance', .350), ('distance_tolerance', .010), ('lateral_tolerance', .010),
            ('hold_time', .5), ('kp_x', 1.), ('kp_y', 1.), ('max_vx', .15), ('max_vy', .10),
            ('align_gate', .04), ('axis_x', 1), ('axis_y', 0), ('axis_yaw', 2),
            ('deadman_button', 9), ('start_button', 3), ('cancel_button', 1),
            ('deadzone', .12), ('override_threshold', .20),
            ('manual_linear', .15), ('manual_angular', .5),
            ('check_obstacles', True), ('half_length', .15), ('half_width', .15), ('obstacle_margin', .025),
            ('association_mode', 'calibrated'), ('validated_min_range', .0),
            ('camera_align_enabled', False), ('camera_align_model', [0.0]),
            ('camera_align_timeout', 4.), ('camera_align_max_speed', .03),
            ('camera_align_tolerance', .012), ('camera_align_settle_time', .20),
            ('auto_enabled', False), ('base_frame', 'base_link'),
            ('require_camera_during_tracking', False),
        ])
        self.p = lambda name: self.get_parameter(name).value
        self.state = 'MANUAL'
        self.reason = ''
        self.joy_at = self.started = self.held_since = None
        self.estimate = self.detection = None
        self.lock_stamp = self.lock_requested = self.acquire_stamp = None
        self.acquire_started = self.camera_centered_since = self.camera_centered_stamp = None
        self.axes = (0., 0., 0.)
        self.deadman = self.start_pressed = self.cancel_pressed = self.start_edge = False
        self.cancel_reason = ''
        self.mux = ControlMux(self)
        self.status = self.create_publisher(String, '/ball/state', 10)
        self.tracking_lock = self.create_publisher(BallEstimate, '/ball/tracking_lock', 10)
        self.create_subscription(Joy, '/joy', self.on_joy, LATEST)
        self.create_subscription(BallDetection, '/ball/detection', self.on_detection, LATEST)
        self.create_subscription(BallEstimate, '/ball/estimate', self.on_estimate, LATEST)
        self.create_timer(1. / self.p('rate'), self.tick)

    def stamped_fresh(self, stamp, max_age):
        return 0 < seconds(stamp) and -.05 <= seconds(self.get_clock().now().to_msg()) - seconds(stamp) < max_age

    def on_joy(self, msg):
        now = monotonic()
        axes = [self.p(name) for name in ('axis_x', 'axis_y', 'axis_yaw')]
        buttons = [self.p(name) for name in ('deadman_button', 'start_button', 'cancel_button')]
        if (not self.stamped_fresh(msg.header.stamp, self.p('joy_timeout'))
                or any(not 0 <= i < len(msg.axes) for i in axes)
                or any(not 0 <= i < len(msg.buttons) for i in buttons)
                or any(not math.isfinite(msg.axes[i]) or abs(msg.axes[i]) > 1 for i in axes)):
            self.joy_at = None
            self.start_edge = False
            if self.state in ACTIVE:
                self.cancel_reason = 'invalid joystick data'
            return
        self.axes = tuple(msg.axes[i] for i in axes)
        deadman, start, cancel = (bool(msg.buttons[i]) for i in buttons)
        connected = self.joy_at is not None and now - self.joy_at < self.p('joy_timeout')
        self.start_edge |= connected and start and not self.start_pressed
        self.joy_at = now
        self.deadman, self.start_pressed, self.cancel_pressed = deadman, start, cancel
        if self.state in ACTIVE:
            if cancel or not deadman:
                self.cancel_reason = 'cancel button or L1 release'
            elif max(map(abs, self.axes)) >= self.p('override_threshold'):
                self.cancel_reason = 'stick override'

    def detection_ready(self, msg):
        return (msg is not None and msg.detected
                and (self.p('association_mode') == 'empirical' or msg.ray.z > 0))

    def on_detection(self, msg):
        self.detection = msg
        if self.state == 'CAMERA_ALIGN' and not self.detection_ready(msg):
            self.cancel_reason = 'ball lost during camera alignment'

    def on_estimate(self, msg):
        self.estimate = msg
        if (self.state in ('ALIGNING', 'APPROACHING') and msg.tracking
                and msg.image_stamp == self.lock_stamp and not msg.valid):
            self.cancel_reason = 'LiDAR target lost or ambiguous'

    def transition(self, state, reason=''):
        if self.state != state:
            self.get_logger().info(f'{self.state} -> {state} {reason}')
        if state in ('ABORT', 'COMPLETE') and self.lock_stamp is not None:
            self.tracking_lock.publish(BallEstimate())
            self.lock_stamp = self.lock_requested = None
        self.state, self.reason = state, reason

    def abort(self, reason):
        self.transition('ABORT', reason)
        return Twist()

    def begin_acquisition(self, now):
        self.acquire_started = now
        self.acquire_stamp = seconds(self.get_clock().now().to_msg())
        self.transition('DETECTING')

    def camera_align(self, now):
        if now - self.started >= self.p('camera_align_timeout'):
            return self.abort('camera alignment time limit')
        est, detection = self.estimate, self.detection
        timeout = self.p('sensor_timeout')
        if est is None or not self.stamped_fresh(est.header.stamp, timeout):
            return self.abort('LiDAR scan unavailable during camera alignment')
        if not self.detection_ready(detection) or not self.stamped_fresh(detection.header.stamp, timeout):
            return self.abort('camera ball unavailable during alignment')
        offset = camera_offset(detection, self.p('camera_align_model'))
        if offset is None:
            return self.abort('ball outside camera alignment range')
        theta, distance = offset
        if abs(theta) <= self.p('camera_align_tolerance'):
            if self.camera_centered_since is None:
                self.camera_centered_since = now
                self.camera_centered_stamp = seconds(detection.header.stamp)
            elif (now - self.camera_centered_since >= self.p('camera_align_settle_time')
                  and seconds(detection.header.stamp) > self.camera_centered_stamp):
                self.begin_acquisition(now)
            return Twist()
        self.camera_centered_since = self.camera_centered_stamp = None
        # The empirical angle uses base coordinates (positive means left).
        max_speed = min(.03, self.p('camera_align_max_speed'), self.p('max_vy'))
        if max_speed <= 0:
            return self.abort('invalid camera alignment speed')
        vy = math.copysign(min(max_speed, 2. * distance * abs(math.tan(theta))), theta)
        if self.p('check_obstacles') and (not est.valid or est.tracking
                or not clearance(est, 0., vy, self.p('half_length'), self.p('half_width'),
                                 self.p('obstacle_margin'))):
            return self.abort('camera alignment clearance unavailable or blocked')
        return velocity(y=vy)

    def automatic(self, now):
        if self.cancel_reason:
            return self.abort(self.cancel_reason)
        if now - self.started >= self.p('approach_timeout'):
            return self.abort('approach time limit')
        if self.state == 'CAMERA_ALIGN':
            return self.camera_align(now)
        est, detection = self.estimate, self.detection
        timeout = self.p('sensor_timeout')
        if self.p('require_camera_during_tracking') and self.state in ('ALIGNING', 'APPROACHING'):
            if not self.detection_ready(detection) or not self.stamped_fresh(detection.header.stamp, timeout):
                return self.abort('camera ball lost or stale')
        fresh_estimate = (est is not None and est.valid and est.header.frame_id == self.p('base_frame')
                          and self.stamped_fresh(est.header.stamp, timeout))
        if self.state == 'DETECTING':
            if (fresh_estimate and not est.tracking and seconds(est.header.stamp) >= self.acquire_stamp
                    and self.detection_ready(detection)
                    and self.stamped_fresh(detection.header.stamp, timeout)
                    and self.stamped_fresh(est.image_stamp, timeout)):
                self.lock_stamp, self.lock_requested = est.image_stamp, now
                self.tracking_lock.publish(est)
                self.transition('ALIGNING' if abs(est.center.y) > self.p('align_gate') else 'APPROACHING')
                return Twist()
            if now - self.acquire_started < self.p('detection_timeout'):
                return Twist()
            return self.abort('camera and LiDAR acquisition failed')
        if not (fresh_estimate and est.tracking and est.image_stamp == self.lock_stamp):
            if now - self.lock_requested < timeout:
                return Twist()
            return self.abort('missing or stale LiDAR track')

        target, tolerance = self.p('target_distance'), self.p('distance_tolerance')
        verified_min = self.p('validated_min_range')
        error = est.surface_distance - target
        lateral = est.center.y
        if (not all(math.isfinite(v) for v in (error, lateral, est.scan_range_min, verified_min))
                or verified_min <= 0 or target < max(est.scan_range_min, verified_min)
                or est.surface_distance < max(est.scan_range_min, verified_min) or error < -tolerance):
            return self.abort('outside verified approach range')
        close = abs(error) <= tolerance
        centered = abs(lateral) <= self.p('lateral_tolerance')
        vx = 0. if close else min(self.p('max_vx'), max(0., self.p('kp_x') * error))
        vy = 0. if centered else max(-self.p('max_vy'),
                                     min(self.p('max_vy'), self.p('kp_y') * lateral))
        aligning = abs(lateral) > self.p('align_gate')
        if aligning:
            vx = 0.
        if self.p('association_mode') == 'empirical':
            # Scan association can use two revolutions in this mode.
            speed = math.hypot(vx, vy)
            if speed > .035:
                vx, vy = vx * .035 / speed, vy * .035 / speed
        if self.p('check_obstacles') and not clearance(
                est, vx, vy, self.p('half_length'), self.p('half_width'), self.p('obstacle_margin')):
            return self.abort('swept footprint blocked')
        self.transition('ALIGNING' if aligning else 'APPROACHING')
        if close and centered:
            if self.held_since is None:
                self.held_since = now
            elif now - self.held_since >= self.p('hold_time'):
                self.transition('COMPLETE')
        else:
            self.held_since = None
        return velocity(vx, vy)

    def tick(self):
        now = monotonic()
        start_edge, self.start_edge = self.start_edge, False
        connected = self.joy_at is not None and now - self.joy_at < self.p('joy_timeout')
        permitted = connected and self.deadman and not self.cancel_pressed
        manual, auto = Twist(), Twist()
        inhibit = not permitted
        if self.state in ('ABORT', 'COMPLETE'):
            self.transition('MANUAL')
            start_edge = False
        if self.state == 'MANUAL':
            neutral = max(map(abs, self.axes)) <= self.p('deadzone')
            if start_edge and permitted and neutral:
                self.cancel_reason = ''
                if self.p('auto_enabled'):
                    self.started, self.held_since = now, None
                    self.tracking_lock.publish(BallEstimate())
                    if self.p('camera_align_enabled') and self.p('association_mode') == 'empirical':
                        self.camera_centered_since = self.camera_centered_stamp = None
                        self.transition('CAMERA_ALIGN')
                    else:
                        self.begin_acquisition(now)
                else:
                    self.abort('auto_enabled is false')
            if permitted and self.state == 'MANUAL':
                deadzone = self.p('deadzone')
                x, y, yaw = [math.copysign(max(0., abs(axis) - deadzone) / (1. - deadzone), axis)
                             for axis in self.axes]
                manual = velocity(x * self.p('manual_linear'), y * self.p('manual_linear'),
                                  yaw * self.p('manual_angular'))
        if self.state in ACTIVE:
            auto = self.automatic(now) if permitted else self.abort(self.cancel_reason or 'joystick timeout or deadman')
        self.mux.publish(manual, auto, self.state in ACTIVE,
                         inhibit or self.state in ('ABORT', 'COMPLETE'))
        self.status.publish(String(data=f'{self.state} {self.reason}'.strip()))

    def destroy_node(self):
        self.tracking_lock.publish(BallEstimate())
        self.mux.publish(Twist(), Twist(), False, True)
        super().destroy_node()


def main():
    run(ApproachController)
