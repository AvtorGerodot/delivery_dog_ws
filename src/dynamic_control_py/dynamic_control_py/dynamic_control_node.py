import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import pinocchio as pin
from ament_index_python.packages import get_package_share_directory
import numpy as np
import os

class CTCController(Node):
    def __init__(self):
        super().__init__('ctc_controller')

        # Загрузка URDF через ament_index (правильный способ в ROS2)
        z1_pkg_share = get_package_share_directory('z1_model')
        urdf_path = os.path.join(z1_pkg_share, 'urdf', 'z1_preview.urdf')
        self.get_logger().info(f'Loading URDF from: {urdf_path}')

        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data  = self.model.createData()
        self.get_logger().info(f'Model loaded: {self.model.nq} DOF')

        # PD-коэффициенты
        self.Kp = np.diag([200.0, 300.0, 200.0, 100.0, 100.0, 100.0, 50])
        self.Kd = np.diag([ 20.0,  30.0,  20.0,  10.0,  10.0,  10.0, 5])

        # Целевая конфигурация (рад) — начальная поза из gazebo.xacro
        self.q_d   = np.array([0.0, 0.0, -0.035, -1.047, 1.0, 0.0, 0.0])
        self.dq_d  = np.zeros(self.model.nq)
        self.ddq_d = np.zeros(self.model.nq)

        # Флаг готовности — не публиковать пока нет первого joint_states
        self.ready = False

        self.sub = self.create_subscription(
            JointState,
            '/z1/joint_states',
            self.joint_states_cb,
            10
        )
        self.pub = self.create_publisher(
            Float64MultiArray,
            '/z1/effort_controller/commands',
            10
        )

    def joint_states_cb(self, msg: JointState):
        # JointState может содержать суставы в произвольном порядке —
        # нужно явно сортировать по имени
        name_to_idx = {name: i for i, name in enumerate(msg.name)}
        joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'jointGripper']

        try:
            q  = np.array([msg.position[name_to_idx[n]] for n in joint_names])
            dq = np.array([msg.velocity[name_to_idx[n]] for n in joint_names])
        except KeyError:
            self.get_logger().warn('Joint names mismatch in JointState message')
            return

        # Вычисление полной динамики
        pin.computeAllTerms(self.model, self.data, q, dq)
        M = self.data.M   # (7x7) матрица инерции
        C = self.data.C   # (7x7) матрица Кориолиса
        g = self.data.g   # (7,)  вектор гравитации

        # CTC закон управления
        e   = self.q_d  - q
        de  = self.dq_d - dq
        tau = M @ (self.ddq_d + self.Kp @ e + self.Kd @ de) + C @ dq + g

        # Clamp — защита от выхода за пределы ±33 Н·м
        tau = np.clip(tau, -33.0, 33.0)

        cmd = Float64MultiArray(data=tau.tolist())
        self.pub.publish(cmd)

        if not self.ready:
            self.get_logger().info('CTC controller running')
            self.ready = True


def main(args=None):
    rclpy.init(args=args)
    node = CTCController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()