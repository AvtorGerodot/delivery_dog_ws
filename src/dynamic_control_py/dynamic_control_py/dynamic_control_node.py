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
    MOVE_TO_PRE = 1  # Движение к точке перед кнопкой (CTC)
    PRESS       = 2  # Импедансное нажатие (движение вперёд с пружиной)
    HOLD        = 3  # Удержание нажатия
    RETRACT     = 4  # Отход назад (CTC)
    FINISH      = 5


class CTCController(Node):
    def __init__(self):
        super().__init__('ctc_controller')

        # ── Загрузка модели ────────────────────────────────────────────────
        z1_share  = get_package_share_directory('z1_model')
        urdf_path = os.path.join(z1_share, 'urdf', 'z1_preview.urdf')
        self.get_logger().info(f'Loading URDF: {urdf_path}')
        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data  = self.model.createData()
        self.get_logger().info(f'Model loaded: {self.model.nq} DOF')

        # ID TCP-фрейма (конец гриппера)
        self.tcp_frame_id = self.model.getFrameId('gripperMover')

        # ── CTC PD-коэффициенты (для свободного движения) ──────────────────
        self.Kp = np.diag([300.0, 400.0, 400.0, 2000.0, 2000.0, 2000.0, 50.0])
        self.Kd = np.diag([ 20.0,  20.0,  30.0,   30.0,   30.0,   30.0,  5.0])

        # ── Импедансные коэффициенты (для нажатия) ─────────────────────────
        # K_imp[0,0] = 80 (мягко по оси X), остальные 500 (жёстко)
        self.K_imp = np.diag([80.0, 500.0, 500.0, 100.0, 100.0, 100.0])
        self.D_imp = np.diag([ 8.0,  50.0,  50.0,  10.0,  10.0,  10.0])

        # ── Целевые точки в ЛОКАЛЬНОЙ системе координат манипулятора ───────
        # Допустим, мы подъехали так, что кнопка находится на x=0.3, z=0.5 от базы манипулятора
        self.button_pos = np.array([0.3, 0.0, 0.5])
        
        # Точка ПЕРЕД кнопкой (отступаем 10 см назад по оси X)
        button_normal = np.array([1.0, 0.0, 0.0]) 
        self.pre_button_pos = self.button_pos - 0.10 * button_normal
        
        # Ориентация TCP (например, стартовая ориентация или просто единичная)
        # Лучше рассчитать стартовую, чтобы IK сошелся легко:
        self.q_start = np.array([0.0, 0.0, -0.06, 0.0, 0.0, 0.0, 0.0])
        pin.forwardKinematics(self.model, self.data, self.q_start)
        pin.updateFramePlacements(self.model, self.data)
        self.button_rot = self.data.oMf[self.tcp_frame_id].rotation.copy()

        # ── Состояние робота ───────────────────────────────────────────────
        self.q_current  = np.copy(self.q_start)
        self.dq_current = np.zeros(self.model.nq)

        # ── Переменные сплайнов ────────────────────────────────────────────
        # Суставной сплайн (для CTC)
        self.q_d   = np.copy(self.q_start)
        self.dq_d  = np.zeros(self.model.nq)
        self.ddq_d = np.zeros(self.model.nq)
        
        # Декартов сплайн (для Импеданса)
        self.x_d  = np.copy(self.pre_button_pos)
        self.dx_d = np.zeros(3)

        # ── Конечный автомат ───────────────────────────────────────────────
        self.state      = State.INIT
        self.state_time = 0.0
        self.timer_period = 0.01

        self.q_ik_seed = np.copy(self.q_start)
        self.q_pre_button = None

        # ── Детектор касания ───────────────────────────────────────────────
        self.effort_window = []
        self.contact_detected = False
        self.bottomed_out = False
        self.STALL_THRESHOLD = 25.0
        self.WINDOW_SIZE = 10

        self.ready = False

        # ── ROS2 ───────────────────────────────────────────────────────────
        self.sub = self.create_subscription(
            JointState, '/robot/joint_states', self.joint_states_cb, 10)
        self.pub = self.create_publisher(
            Float64MultiArray, '/robot/effort_controller/commands', 10)

        self.timer = self.create_timer(self.timer_period, self.control_loop)


    def solve_ik(self, target_pos, target_rot, q_seed, max_iter=1000, eps=1e-3, lam=1e-2):
        """ ОЗК методом DLS. Возвращает (q, success) """
        q = np.copy(q_seed)
        oMdes = pin.SE3(target_rot, target_pos)

        for i in range(max_iter):
            pin.forwardKinematics(self.model, self.data, q)
            pin.updateFramePlacements(self.model, self.data)
            oMf = self.data.oMf[self.tcp_frame_id]
            
            err = pin.log(oMdes.actInv(oMf)).vector
            if np.linalg.norm(err) < eps:
                self.get_logger().info(f'IK converged in {i} iterations')
                return q, True

            J = pin.computeFrameJacobian(
                self.model, self.data, q, self.tcp_frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
            
            JJT = J @ J.T
            dq = J.T @ np.linalg.solve(JJT + lam**2 * np.eye(6), err)
            q = pin.integrate(self.model, q, -dq * 0.5)

        return q, False

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

        self.q_current  = q
        self.dq_current = dq

        if self.state == State.PRESS:
            self.update_contact_detection(tau_meas)

        pin.computeAllTerms(self.model, self.data, q, dq)
        M = self.data.M
        C = self.data.C
        g = self.data.g

        # Выбор закона управления
        if self.state in [State.PRESS, State.HOLD]:
            tau = self._impedance_control(q, dq, M, C, g)
        else:
            e   = self.q_d   - q
            de  = self.dq_d  - dq
            tau = M @ (self.ddq_d + self.Kp @ e + self.Kd @ de) + C @ dq + g

        tau[6] = 0.0 # Гриппер расслаблен
        tau = np.clip(tau, -33.0, 33.0)
        self.pub.publish(Float64MultiArray(data=tau.tolist()))

        if not self.ready:
            self.get_logger().info('CTC controller running')
            self.ready = True


    def _impedance_control(self, q, dq, M, C, g):
        pin.forwardKinematics(self.model, self.data, q, dq)
        pin.updateFramePlacements(self.model, self.data)
        
        oMf = self.data.oMf[self.tcp_frame_id]
        x_cur = oMf.translation 

        J = pin.computeFrameJacobian(
            self.model, self.data, q, self.tcp_frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        
        # Исправлено: dq имеет размер 7, умножаем на всю J
        dx_cur = J[:3, :] @ dq 
        
        # Ошибка в декартовом пространстве (отслеживаем декартов сплайн x_d)
        x_err  = self.x_d - x_cur
        dx_err = self.dx_d - dx_cur

        F_pos = self.K_imp[:3, :3] @ x_err + self.D_imp[:3, :3] @ dx_err
        F_task = np.concatenate([F_pos, np.zeros(3)])

        tau = J.T @ F_task + g
        return tau


    def get_spline(self, x_start, x_end, T, t):
        """Универсальный полином (подходит и для векторов q, и для декартовых x)"""
        if t <= 0: return np.copy(x_start), np.zeros_like(x_start), np.zeros_like(x_start)
        if t >= T: return np.copy(x_end), np.zeros_like(x_end), np.zeros_like(x_end)
        
        tau = t / T
        s   = 10*tau**3 - 15*tau**4 + 6*tau**5
        ds  = (30*tau**2 - 60*tau**3 + 30*tau**4) / T
        dds = (60*tau - 180*tau**2 + 120*tau**3) / (T**2)
        
        delta = x_end - x_start
        return x_start + delta*s, delta*ds, delta*dds


    def control_loop(self):
        if not self.ready: return
        self.state_time += self.timer_period

        # ── INIT ────────────────────────────────────────
        if self.state == State.INIT:
            self.q_d, self.dq_d, self.ddq_d = self.get_spline(self.q_start, self.q_start, 2.0, self.state_time)
            if self.state_time > 2.0:
                q_sol, ok = self.solve_ik(self.pre_button_pos, self.button_rot, self.q_ik_seed)
                if ok:
                    self.q_pre_button = q_sol
                    self.transition_to(State.MOVE_TO_PRE)
                else:
                    self.get_logger().error('IK failed! Point is unreachable. SHUTTING DOWN.')
                    self.shutdown_node()

        # ── MOVE_TO_PRE (Суставной сплайн) ────────────────
        elif self.state == State.MOVE_TO_PRE:
            self.q_d, self.dq_d, self.ddq_d = self.get_spline(self.q_start, self.q_pre_button, 3.0, self.state_time)
            if self.state_time > 3.5:
                self.bottomed_out = False
                self.effort_window = []
                self.transition_to(State.PRESS)

        # ── PRESS (Декартов сплайн + Импеданс) ────────────
        elif self.state == State.PRESS:
            # Движемся от pre_button_pos к button_pos по прямой за 2 секунды
            self.x_d, self.dx_d, _ = self.get_spline(self.pre_button_pos, self.button_pos, 2.0, self.state_time)
            
            if self.bottomed_out:
                self.transition_to(State.HOLD)
            elif self.state_time > 5.0:
                self.get_logger().warn('Timeout. Retracting.')
                self.transition_to(State.RETRACT)

        # ── HOLD ──────────────────────────────────────────
        elif self.state == State.HOLD:
            if self.state_time > 0.5:
                self.transition_to(State.RETRACT)

        # ── RETRACT (Суставной сплайн) ────────────────────
        elif self.state == State.RETRACT:
            self.q_d, self.dq_d, self.ddq_d = self.get_spline(self.q_current, self.q_start, 2.0, self.state_time)
            if self.state_time > 2.0:
                self.transition_to(State.FINISH)

        # ── FINISH ────────────────────────────────────────
        elif self.state == State.FINISH:
            self.get_logger().info('Task complete! Shutting down node.')
            self.shutdown_node()


    def transition_to(self, new_state: State):
        self.get_logger().info(f'{self.state.name} → {new_state.name}')
        self.state = new_state
        self.state_time = 0.0
        
    def shutdown_node(self):
        """Безопасное завершение работы ноды"""
        # Посылаем нулевые моменты перед выключением (чтобы рука мягко упала/замерла)
        self.pub.publish(Float64MultiArray(data=np.zeros(7).tolist()))
        self.timer.cancel()
        raise SystemExit # Инициирует выход из rclpy.spin


def main(args=None):
    rclpy.init(args=args)
    node = CTCController()
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