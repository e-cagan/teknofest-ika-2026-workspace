"""
Keyboard Teleop Node — TEKNOFEST 2026 İKA

Terminal üzerinden klavye ile araç kontrolü.

Kontroller:
    W/S     : İleri / Geri
    A/D     : Sola / Sağa dön
    Q/E     : Sol-ileri / Sağ-ileri (çapraz)
    Z/C     : Sol-geri / Sağ-geri (çapraz)

    I/K     : Gimbal tilt yukarı/aşağı
    J/L     : Gimbal pan sol/sağ
    F       : Lazer fire

    T       : Turbo toggle
    H       : Far toggle
    X       : E-Stop toggle
    SPACE   : Acil dur (hız sıfırla)

    R/V     : Hız artır / azalt
    CTRL+C  : Çıkış

Publish:
    /cmd_vel_teleop         — Twist
    /targeting/gimbal_cmd   — Vector3
    /targeting/fire_request — Bool
    /safety/estop           — Bool
    /vehicle/headlights     — Bool
"""

import sys
import termios
import tty
import select
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Bool


BANNER = """
╔══════════════════════════════════════════════╗
║       İKA KEYBOARD TELEOP KONTROLÜ          ║
╠══════════════════════════════════════════════╣
║  Sürüş:                                     ║
║    W/S : İleri/Geri    A/D : Sola/Sağa      ║
║    Q/E : Çapraz ileri  Z/C : Çapraz geri     ║
║    SPACE : Acil dur                          ║
║                                              ║
║  Gimbal:                                     ║
║    I/K : Tilt yukari/aşağı                   ║
║    J/L : Pan sol/sağ                         ║
║    F   : Lazer fire                          ║
║                                              ║
║  Sistem:                                     ║
║    X : E-Stop toggle   H : Far toggle        ║
║    T : Turbo toggle                          ║
║    R/V : Hız artır/azalt                     ║
║    CTRL+C : Çıkış                            ║
╚══════════════════════════════════════════════╝
"""

# Tuş → (linear.x çarpanı, angular.z çarpanı)
MOVE_BINDINGS = {
    'w': (1.0, 0.0),     # ileri
    's': (-1.0, 0.0),    # geri
    'a': (0.0, 1.0),     # sola dön
    'd': (0.0, -1.0),    # sağa dön
    'q': (1.0, 1.0),     # sol-ileri
    'e': (1.0, -1.0),    # sağ-ileri
    'z': (-1.0, 1.0),    # sol-geri
    'c': (-1.0, -1.0),   # sağ-geri
}

# Gimbal tuşları → (pan çarpanı, tilt çarpanı)
GIMBAL_BINDINGS = {
    'j': (1.0, 0.0),     # pan sol
    'l': (-1.0, 0.0),    # pan sağ
    'i': (0.0, 1.0),     # tilt yukarı
    'k': (0.0, -1.0),    # tilt aşağı
}


class TeleopKeyboardNode(Node):

    def __init__(self):
        super().__init__('teleop_keyboard_node')

        # ── Parametreler ──
        self.declare_parameter('linear_speed', 0.5)
        self.declare_parameter('angular_speed', 1.0)
        self.declare_parameter('linear_step', 0.1)
        self.declare_parameter('angular_step', 0.2)

        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.linear_step = self.get_parameter('linear_step').value
        self.angular_step = self.get_parameter('angular_step').value

        self.gimbal_speed = 0.3  # rad/s

        # ── Durum ──
        self.turbo = False
        self.estop_active = False
        self.headlights_on = False

        # ── Publishers ──
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_teleop', 10)
        self.gimbal_pub = self.create_publisher(Vector3, '/targeting/gimbal_cmd', 10)
        self.estop_pub = self.create_publisher(Bool, '/safety/estop', 10)
        self.fire_pub = self.create_publisher(Bool, '/targeting/fire_request', 10)
        self.headlight_pub = self.create_publisher(Bool, '/vehicle/headlights', 10)

        # ── Periyodik cmd_vel yayını (tuş basılı olmasa bile son durumu gönder) ──
        self.current_linear = 0.0
        self.current_angular = 0.0
        self.key_active = False
        self.publish_timer = self.create_timer(0.05, self._publish_cmd)  # 20 Hz

        self.get_logger().info('Keyboard teleop başlatıldı')

    def _publish_cmd(self):
        """Periyodik cmd_vel yayını."""
        cmd = Twist()
        if self.key_active:
            cmd.linear.x = self.current_linear
            cmd.angular.z = self.current_angular
        else:
            # Tuş bırakıldığında dur
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)

    def run(self):
        """Ana klavye okuma döngüsü."""
        print(BANNER)
        self._print_status()

        # Terminal ayarlarını kaydet
        old_settings = termios.tcgetattr(sys.stdin)

        try:
            # Terminal'i raw moda al (karakter karakter oku)
            tty.setcbreak(sys.stdin.fileno())

            while rclpy.ok():
                # Non-blocking tuş okuma
                if select.select([sys.stdin], [], [], 0.05)[0]:
                    key = sys.stdin.read(1).lower()
                    self._handle_key(key)
                else:
                    self.key_active = False

                # ROS callback'leri çalıştır
                rclpy.spin_once(self, timeout_sec=0)

        except KeyboardInterrupt:
            pass
        finally:
            # Motorları durdur
            stop = Twist()
            self.cmd_vel_pub.publish(stop)
            # Terminal ayarlarını geri yükle
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
            print('\nKapatıldı.')

    def _handle_key(self, key: str):
        """Tuş girişini işle."""

        # ── Sürüş ──
        if key in MOVE_BINDINGS:
            lin_mult, ang_mult = MOVE_BINDINGS[key]
            speed_mult = 2.0 if self.turbo else 1.0
            self.current_linear = lin_mult * self.linear_speed * speed_mult
            self.current_angular = ang_mult * self.angular_speed * speed_mult
            self.key_active = True
            return

        # ── Gimbal ──
        if key in GIMBAL_BINDINGS:
            pan_mult, tilt_mult = GIMBAL_BINDINGS[key]
            gimbal_msg = Vector3()
            gimbal_msg.x = pan_mult * self.gimbal_speed
            gimbal_msg.y = tilt_mult * self.gimbal_speed
            gimbal_msg.z = 0.0
            self.gimbal_pub.publish(gimbal_msg)
            return

        # ── SPACE — acil dur ──
        if key == ' ':
            self.current_linear = 0.0
            self.current_angular = 0.0
            self.key_active = False
            print('\r\033[K[STOP] Motorlar durduruldu')
            return

        # ── F — lazer fire ──
        if key == 'f':
            fire_msg = Bool()
            fire_msg.data = True
            self.fire_pub.publish(fire_msg)
            print('\r\033[K[FIRE] Lazer ateşlendi!')
            return

        # ── X — e-stop toggle ──
        if key == 'x':
            self.estop_active = not self.estop_active
            estop_msg = Bool()
            estop_msg.data = self.estop_active
            self.estop_pub.publish(estop_msg)
            if self.estop_active:
                self.current_linear = 0.0
                self.current_angular = 0.0
                self.key_active = False
            state = "AKTİF" if self.estop_active else "SERBEST"
            print(f'\r\033[K[E-STOP] {state}')
            return

        # ── H — far toggle ──
        if key == 'h':
            self.headlights_on = not self.headlights_on
            hl_msg = Bool()
            hl_msg.data = self.headlights_on
            self.headlight_pub.publish(hl_msg)
            state = "AÇIK" if self.headlights_on else "KAPALI"
            print(f'\r\033[K[FAR] {state}')
            return

        # ── T — turbo toggle ──
        if key == 't':
            self.turbo = not self.turbo
            state = "AÇIK" if self.turbo else "KAPALI"
            print(f'\r\033[K[TURBO] {state}')
            return

        # ── R/V — hız artır/azalt ──
        if key == 'r':
            self.linear_speed += self.linear_step
            self.angular_speed += self.angular_step
            self._print_status()
            return
        if key == 'v':
            self.linear_speed = max(0.1, self.linear_speed - self.linear_step)
            self.angular_speed = max(0.2, self.angular_speed - self.angular_step)
            self._print_status()
            return

    def _print_status(self):
        """Mevcut hız ayarlarını göster."""
        print(f'\r\033[K  Hız: lin={self.linear_speed:.1f} m/s, '
              f'ang={self.angular_speed:.1f} rad/s | '
              f'Turbo: {"ON" if self.turbo else "OFF"} | '
              f'E-Stop: {"ON" if self.estop_active else "OFF"}', end='')


def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyboardNode()
    try:
        node.run()
    except Exception as e:
        print(f'Hata: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
