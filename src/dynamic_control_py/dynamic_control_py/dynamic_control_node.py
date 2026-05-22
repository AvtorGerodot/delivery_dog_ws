import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import pinocchio as pin
from ament_index_python.packages import get_package_share_directory
import numpy as np
import os
from enum import Enum


class State(Enum):
    INIT        = 0
    MOVE_TO_PRE = 1  # Движение к точке перед кнопкой (Импеданс)
    PRESS       = 2  # Нажатие (Импеданс)
    HOLD        = 3  # Удержание (Импеданс)
    RETRACT     = 4  # Отход назад (Импеданс)
    FINISH      = 5


class ImpedanceController(Node):
    def __init__(self):
        super().__init__('impedance_controller')

        # ── Загрузка модели ────────────────────────────────────────────────
        z1_share  = get_package_share_directory('z1_model')
        urdf_path = os.path.join(z1_share, 'urdf', 'z1_standalone_preview.urdf')
        self.get_logger().info(f'Loading URDF: {urdf_path}')
        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data  = self.model.createData()
        self.get_logger().info(f'Model loaded: {self.model.nq} DOF')

        # ID TCP-фрейма (конец гриппера)
        # self.tcp_frame_id = self.model.getFrameId('gripperMover')
        self.tcp_frame_id = self.model.getFrameId('finger_tcp_link')

        # ── Импедансные коэффициенты (для всего движения) ──────────────────
        # Теперь коэффициенты должны быть достаточно высокими для точного 
        # позиционирования в воздухе, но податливыми по оси X при нажатии.
        # [X, Y, Z, Roll, Pitch, Yaw]
        self.K_imp = np.diag([200.0, 500.0, 500.0, 100.0, 100.0, 100.0])
        self.D_imp = np.diag([ 20.0,  50.0,  50.0,  10.0,  10.0,  10.0])

        # ── Целевые точки в ЛОКАЛЬНОЙ системе координат манипулятора ───────
        # self.button_pos = np.array([0.35, 0.0, 0.5])
        self.button_pos = np.array([0.35, 0.0, 0.6085]) # - 0.11675 - 0.5 + 0.387
        
        # Точка ПЕРЕД кнопкой (отступаем 10 см назад по оси X)
        button_normal = np.array([1.0, 0.0, 0.0]) 
        # self.pre_button_pos = self.button_pos - 0.10 * button_normal
        self.pre_button_pos = self.button_pos - 0.03 * button_normal
        
        # Стартовая поза для инициализации
        self.q_start = np.array([0.0, 0.0, -0.06, 0.0, 0.0, 0.0, 0.0])
        pin.forwardKinematics(self.model, self.data, self.q_start)
        pin.updateFramePlacements(self.model, self.data)
        
        # Запоминаем декартову стартовую позицию TCP
        oMf_start = self.data.oMf[self.tcp_frame_id]
        self.start_pos = oMf_start.translation.copy()
        
        # В чистом импедансном управлении нам нужно удерживать и ориентацию.
        # Запоминаем целевую ориентацию (пусть она будет равна стартовой)
        self.target_rot = oMf_start.rotation.copy()

        # ── Декартов сплайн (Желаемая траектория TCP) ──────────────────────
        self.x_d  = np.copy(self.start_pos)
        self.dx_d = np.zeros(3)

        # ── Конечный автомат ───────────────────────────────────────────────
        self.state      = State.INIT
        self.state_time = 0.0
        self.timer_period = 0.01

        # ── Детектор касания ───────────────────────────────────────────────
        self.effort_window = []
        self.contact_detected = False
        self.bottomed_out = False
        self.STALL_THRESHOLD = 15.0 # Понизил для лучшей чувствительности
        self.WINDOW_SIZE = 10

        self.ready = False

        # ── ROS2 ───────────────────────────────────────────────────────────
        self.sub = self.create_subscription(
            JointState, '/robot/joint_states', self.joint_states_cb, 10)
        self.pub = self.create_publisher(
            Float64MultiArray, '/robot/effort_controller/commands', 10)

        self.timer = self.create_timer(self.timer_period, self.control_loop)


    def update_contact_detection(self, tau_measured):
        contact_force_proxy = np.linalg.norm(tau_measured[3:6])
        self.effort_window.append(contact_force_proxy)
        if len(self.effort_window) > self.WINDOW_SIZE:
            self.effort_window.pop(0)

        mean_effort = np.mean(self.effort_window)
        if mean_effort > self.STALL_THRESHOLD and not self.bottomed_out:
            self.bottomed_out = True
            self.get_logger().info(f'BUTTON PRESSED! Mean effort = {mean_effort:.2f} Nm')


    def joint_states_cb(self, msg: JointState):
        name_to_idx = {name: i for i, name in enumerate(msg.name)}
        joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'jointGripper']
        
        try:
            q  = np.array([msg.position[name_to_idx[n]] for n in joint_names])
            dq = np.array([msg.velocity[name_to_idx[n]] for n in joint_names])
            tau_meas = np.array([msg.effort[name_to_idx[n]] for n in joint_names])
        except KeyError:
            return

        if self.state == State.PRESS:
            self.update_contact_detection(tau_meas)

        # Вычисляем динамику (в основном ради вектора гравитации g)
        pin.computeAllTerms(self.model, self.data, q, dq)
        g = self.data.g

        # Полностью импедансное управление на всём протяжении
        tau = self._impedance_control(q, dq, g)

        tau[6] = 0.0 # Гриппер расслаблен
        tau = np.clip(tau, -33.0, 33.0)
        self.pub.publish(Float64MultiArray(data=tau.tolist()))

        if not self.ready:
            self.get_logger().info('Impedance controller running')
            self.ready = True


    def _impedance_control(self, q, dq, g):
        """ Чистое импедансное управление в декартовом пространстве """
        pin.forwardKinematics(self.model, self.data, q, dq)
        pin.updateFramePlacements(self.model, self.data)
        
        oMf = self.data.oMf[self.tcp_frame_id]
        x_cur = oMf.translation 
        rot_cur = oMf.rotation

        # Якобиан (6x7)
        J = pin.computeFrameJacobian(
            self.model, self.data, q, self.tcp_frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        
        # Скорость TCP
        dx_cur = (J @ dq)[:3]
        
        # 1. Линейная ошибка
        x_err  = self.x_d - x_cur
        dx_err = self.dx_d - dx_cur
        F_pos = self.K_imp[:3, :3] @ x_err + self.D_imp[:3, :3] @ dx_err

        # 2. Угловая ошибка (Ориентация)
        # pin.log3 возвращает вектор ошибки между двумя матрицами вращения
        rot_err = pin.log3(self.target_rot @ rot_cur.T) 
        # Угловая скорость TCP (последние 3 элемента J @ dq)
        w_cur = (J @ dq)[3:]
        w_err = np.zeros(3) - w_cur # Желаемая угловая скорость = 0
        
        F_rot = self.K_imp[3:, 3:] @ rot_err + self.D_imp[3:, 3:] @ w_err

        # Полный вектор силы/момента в декартовом пространстве (6D)
        F_task = np.concatenate([F_pos, F_rot])

        # Пересчёт в моменты суставов + компенсация гравитации
        tau = J.T @ F_task + g
        
        # Опционально: добавление слабого демпфирования в нулевом пространстве 
        # (чтобы локти не "болтались", так как Z1 - избыточный манипулятор)
        # N = np.eye(7) - np.linalg.pinv(J) @ J
        # tau += N @ (-5.0 * dq)
        
        return tau


    def get_spline(self, x_start, x_end, T, t):
        """ Генерация декартового сплайна """
        if t <= 0: return np.copy(x_start), np.zeros_like(x_start)
        if t >= T: return np.copy(x_end), np.zeros_like(x_end)
        
        tau = t / T
        s   = 10*tau**3 - 15*tau**4 + 6*tau**5
        ds  = (30*tau**2 - 60*tau**3 + 30*tau**4) / T
        
        delta = x_end - x_start
        return x_start + delta*s, delta*ds


    def control_loop(self):
        if not self.ready: return
        self.state_time += self.timer_period

        # ── INIT (Удержание начальной позы) ──────────────────
        if self.state == State.INIT:
            self.x_d, self.dx_d = self.get_spline(self.start_pos, self.start_pos, 2.0, self.state_time)
            if self.state_time > 2.0:
                self.transition_to(State.MOVE_TO_PRE)

        # ── MOVE_TO_PRE (Декартов полет к кнопке) ────────────
        elif self.state == State.MOVE_TO_PRE:
            self.x_d, self.dx_d = self.get_spline(self.start_pos, self.pre_button_pos, 3.0, self.state_time)
            if self.state_time > 3.5:
                self.bottomed_out = False
                self.effort_window = []
                self.transition_to(State.PRESS)

        # ── PRESS (Надавливание) ─────────────────────────────
        elif self.state == State.PRESS:
            # Мягкая пружина K_imp[0,0]=80 давит вперёд
            self.x_d, self.dx_d = self.get_spline(self.pre_button_pos, self.button_pos, 2.0, self.state_time)
            
            if self.bottomed_out:
                self.transition_to(State.HOLD)
            elif self.state_time > 5.0:
                self.get_logger().warn('Timeout. Retracting.')
                self.transition_to(State.RETRACT)

        # ── HOLD (Удержание) ─────────────────────────────────
        elif self.state == State.HOLD:
            if self.state_time > 0.5:
                self.transition_to(State.RETRACT)

        # ── RETRACT (Отлет назад) ────────────────────────────
        elif self.state == State.RETRACT:
            # Возвращаемся в pre_button_pos, а не в start_pos, для простоты
            self.x_d, self.dx_d = self.get_spline(self.button_pos, self.pre_button_pos, 2.0, self.state_time)
            if self.state_time > 2.0:
                self.transition_to(State.FINISH)

        # ── FINISH ───────────────────────────────────────────
        elif self.state == State.FINISH:
            self.get_logger().info('Task complete! Shutting down node.')
            self.shutdown_node()


    def transition_to(self, new_state: State):
        self.get_logger().info(f'{self.state.name} → {new_state.name}')
        self.state = new_state
        self.state_time = 0.0
        
    def shutdown_node(self):
        self.pub.publish(Float64MultiArray(data=np.zeros(7).tolist()))
        self.timer.cancel()
        raise SystemExit


def main(args=None):
    rclpy.init(args=args)
    node = ImpedanceController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()