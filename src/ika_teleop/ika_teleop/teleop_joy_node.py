"""
Joystick Teleop Node — TEKNOFEST 2026 İKA

Joystick → /cmd_vel_teleop
Gimbal kontrol → /targeting/gimbal_cmd
Lazer fire → /targeting/fire_request
Headlight toggle → /vehicle/headlights
E-stop → /safety/estop

Logitech F710 / Xbox Controller Layout:
  Sol analog Y  → ileri/geri
  Sol analog X  → sağa/sola dönüş
  Sağ analog X  → gimbal pan
  Sağ analog Y  → gimbal tilt
  RB            → turbo mod (basılı tutarken)
  LB            → e-stop toggle
  A             → lazer fire
  Y             → far toggle
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool


class TeleopJoyNode(Node):

    def __init__(self):
        super().__init__('teleop_joy_node')

        # ── Parametreler ──
        self.declare_parameter('linear_axis', 1)
        self.declare_parameter('angular_axis', 0)
        self.declare_parameter('turbo_button', 5)
        self.declare_parameter('estop_button', 4)
        self.declare_parameter('fire_button', 0)
        self.declare_parameter('headlight_button', 3)
        self.declare_parameter('gimbal_pan_axis', 3)
        self.declare_parameter('gimbal_tilt_axis', 4)
        self.declare_parameter('gimbal_speed', 0.5)
        self.declare_parameter('normal_linear', 0.5)
        self.declare_parameter('normal_angular', 1.0)
        self.declare_parameter('turbo_linear', 1.5)
        self.declare_parameter('turbo_angular', 2.5)
        self.declare_parameter('deadzone', 0.1)

        self.lin_axis = self.get_parameter('linear_axis').value
        self.ang_axis = self.get_parameter('angular_axis').value
        self.turbo_btn = self.get_parameter('turbo_button').value
        self.estop_btn = self.get_parameter('estop_button').value
        self.fire_btn = self.get_parameter('fire_button').value
        self.headlight_btn = self.get_parameter('headlight_button').value
        self.pan_axis = self.get_parameter('gimbal_pan_axis').value
        self.tilt_axis = self.get_parameter('gimbal_tilt_axis').value
        self.gimbal_speed = self.get_parameter('gimbal_speed').value

        self.normal_lin = self.get_parameter('normal_linear').value
        self.normal_ang = self.get_parameter('normal_angular').value
        self.turbo_lin = self.get_parameter('turbo_linear').value
        self.turbo_ang = self.get_parameter('turbo_angular').value
        self.deadzone = self.get_parameter('deadzone').value

        # ── Durum ──
        self.estop_active = False
        self.headlights_on = False
        self.prev_estop_pressed = False
        self.prev_headlight_pressed = False
        self.prev_fire_pressed = False

        # ── Subscriber ──
        self.joy_sub = self.create_subscription(Joy, '/joy', self._joy_callback, 10)

        # ── Publishers ──
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_teleop', 10)
        self.gimbal_pub = self.create_publisher(Vector3, '/targeting/gimbal_cmd', 10)
        self.estop_pub = self.create_publisher(Bool, '/safety/estop', 10)
        self.fire_pub = self.create_publisher(Bool, '/targeting/fire_request', 10)
        self.headlight_pub = self.create_publisher(Bool, '/vehicle/headlights', 10)

        self.get_logger().info('Joystick teleop başlatıldı')

    def _apply_deadzone(self, value: float) -> float:
        """Deadzone uygula — küçük titreşimleri filtrele."""
        if abs(value) < self.deadzone:
            return 0.0
        # Deadzone sonrası 0'dan başlasın diye rescale
        sign = 1.0 if value > 0 else -1.0
        return sign * (abs(value) - self.deadzone) / (1.0 - self.deadzone)

    def _get_axis(self, joy_msg: Joy, axis: int) -> float:
        """Güvenli axis okuma."""
        if axis < len(joy_msg.axes):
            return joy_msg.axes[axis]
        return 0.0

    def _get_button(self, joy_msg: Joy, button: int) -> bool:
        """Güvenli button okuma."""
        if button < len(joy_msg.buttons):
            return bool(joy_msg.buttons[button])
        return False

    def _joy_callback(self, msg: Joy):
        # ── E-Stop toggle (LB — rising edge) ──
        estop_pressed = self._get_button(msg, self.estop_btn)
        if estop_pressed and not self.prev_estop_pressed:
            self.estop_active = not self.estop_active
            estop_msg = Bool()
            estop_msg.data = self.estop_active
            self.estop_pub.publish(estop_msg)
            state = "AKTİF" if self.estop_active else "SERBEST"
            self.get_logger().warn(f'E-STOP {state}')
        self.prev_estop_pressed = estop_pressed

        # ── Far toggle (Y — rising edge) ──
        headlight_pressed = self._get_button(msg, self.headlight_btn)
        if headlight_pressed and not self.prev_headlight_pressed:
            self.headlights_on = not self.headlights_on
            hl_msg = Bool()
            hl_msg.data = self.headlights_on
            self.headlight_pub.publish(hl_msg)
        self.prev_headlight_pressed = headlight_pressed

        # ── Lazer fire (A — rising edge) ──
        fire_pressed = self._get_button(msg, self.fire_btn)
        if fire_pressed and not self.prev_fire_pressed:
            fire_msg = Bool()
            fire_msg.data = True
            self.fire_pub.publish(fire_msg)
            self.get_logger().info('Lazer fire komutu gönderildi')
        self.prev_fire_pressed = fire_pressed

        # ── Turbo mod ──
        turbo = self._get_button(msg, self.turbo_btn)
        max_lin = self.turbo_lin if turbo else self.normal_lin
        max_ang = self.turbo_ang if turbo else self.normal_ang

        # ── Sürüş (sol analog) ──
        linear = self._apply_deadzone(self._get_axis(msg, self.lin_axis)) * max_lin
        angular = self._apply_deadzone(self._get_axis(msg, self.ang_axis)) * max_ang

        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        self.cmd_vel_pub.publish(cmd)

        # ── Gimbal (sağ analog) ──
        pan = self._apply_deadzone(self._get_axis(msg, self.pan_axis)) * self.gimbal_speed
        tilt = self._apply_deadzone(self._get_axis(msg, self.tilt_axis)) * self.gimbal_speed

        if abs(pan) > 0.0 or abs(tilt) > 0.0:
            gimbal_msg = Vector3()
            gimbal_msg.x = pan    # pan hızı (rad/s)
            gimbal_msg.y = tilt   # tilt hızı (rad/s)
            gimbal_msg.z = 0.0
            self.gimbal_pub.publish(gimbal_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopJoyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
