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
    MOVE_TO_PRE = 1  # Движение к точке перед кнопкой (ОЗК → сплайн)
    PRESS       = 2  # Импедансное нажатие
    HOLD        = 3  # Удержание нажатия
    RETRACT     = 4  # Отход назад
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

        # ID TCP-фрейма (конец гриппера — последний фрейм в URDF)
        # Убедитесь, что это имя совпадает с вашим URDF
        self.tcp_frame_id = self.model.getFrameId('gripperMover')

        # ── CTC PD-коэффициенты ────────────────────────────────────────────
        self.Kp = np.diag([300.0, 400.0, 400.0, 2000.0, 2000.0, 2000.0, 50.0])
        self.Kd = np.diag([ 20.0,  20.0,  30.0,   30.0,   30.0,   30.0,  5.0])

        # ── Импедансные коэффициенты в пространстве задач (6D: xyz + rpy) ──
        # Жёсткость только по оси нажатия (ось X TCP), остальные — большие
        # чтобы рука не "гуляла" в поперечных направлениях
        self.K_imp = np.diag([80.0,  500.0, 500.0, 100.0, 100.0, 100.0])
        self.D_imp = np.diag([ 8.0,   50.0,  50.0,  10.0,  10.0,  10.0])
        # Индекс 0 = ось нажатия (нужно подобрать под ориентацию вашего TCP)

        # ── Целевые точки (заполнить через solve_ik!) ─────────────────────
        # BUTTON_POS — координаты кнопки в мировой СК
        # Измерьте в вашем .sdf мире
        self.button_pos = np.array([0.5, 0.0, 1.2])   # ← ваши координаты
        self.button_rot = np.eye(3)                     # Ориентация TCP при нажатии

        # PRE_BUTTON_POS — точка в 10 см ПЕРЕД кнопкой по нормали к панели
        button_normal = np.array([1.0, 0.0, 0.0])       # Нормаль кнопочной панели
        self.pre_button_pos = self.button_pos - 0.10 * button_normal

        # Начальная поза (в суставных координатах)
        self.q_start = np.array([0.0, 0.0, -0.06, 0.0, 0.0, 0.0, 0.0])

        # ── Состояние робота (заполняется в joint_states_cb) ──────────────
        self.q_current  = np.copy(self.q_start)
        self.dq_current = np.zeros(self.model.nq)

        # ── Желаемая траектория (обновляется в control_loop) ──────────────
        self.q_d   = np.copy(self.q_start)
        self.dq_d  = np.zeros(self.model.nq)
        self.ddq_d = np.zeros(self.model.nq)

        # ── Конечный автомат ──────────────────────────────────────────────
        self.state      = State.INIT
        self.state_time = 0.0
        self.timer_period = 0.01   # 100 Гц

        # Начальная конфигурация для ОЗК (тёплый старт = текущая поза)
        self.q_ik_seed = np.copy(self.q_start)

        # Конфигурация, в которую едем (результат ОЗК)
        self.q_pre_button = None   # Будет вычислен в INIT

        # ── Детектор касания ──────────────────────────────────────────────
        self.effort_window = []         # Скользящее окно моментов
        self.contact_detected   = False
        self.bottomed_out       = False
        self.CONTACT_THRESHOLD  = 3.0   # Н·м — порог обнаружения касания
        self.STALL_THRESHOLD    = 25.0  # Н·м — порог "кнопка нажата до упора"
        self.WINDOW_SIZE        = 10    # Кол-во тиков для скользящего среднего

        # ── Флаг готовности ───────────────────────────────────────────────
        self.ready = False

        # ── ROS2 топики ───────────────────────────────────────────────────
        self.sub = self.create_subscription(
            JointState, '/robot/joint_states', self.joint_states_cb, 10)
        self.pub = self.create_publisher(
            Float64MultiArray, '/robot/effort_controller/commands', 10)

        self.timer = self.create_timer(self.timer_period, self.control_loop)

    # ══════════════════════════════════════════════════════════════════════
    # ПРЯМАЯ ЗАДАЧА КИНЕМАТИКИ
    # Возвращает позицию и матрицу вращения TCP для текущей конфигурации q
    # ══════════════════════════════════════════════════════════════════════
    def forward_kinematics(self, q: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """
        Возвращает (position: [3], rotation: [3x3]) TCP в мировых координатах.
        """
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        oMf = self.data.oMf[self.tcp_frame_id]
        return oMf.translation.copy(), oMf.rotation.copy()

    # ══════════════════════════════════════════════════════════════════════
    # ОБРАТНАЯ ЗАДАЧА КИНЕМАТИКИ — Damped Least Squares
    # Находит q такое, что TCP оказывается в (target_pos, target_rot)
    # ══════════════════════════════════════════════════════════════════════
    def solve_ik(self,
                 target_pos: np.ndarray,
                 target_rot: np.ndarray,
                 q_seed: np.ndarray,
                 max_iter: int = 1000,
                 eps: float = 1e-3,
                 lam: float = 1e-2) -> tuple[np.ndarray, bool]:
        """
        Итеративная ОЗК методом Damped Least Squares.

        Args:
            target_pos: Целевая позиция TCP [x, y, z]
            target_rot: Целевая ориентация TCP [3x3]
            q_seed:     Начальное приближение
            max_iter:   Макс. число итераций
            eps:        Порог сходимости (норма ошибки)
            lam:        Коэффициент демпфирования DLS

        Returns:
            (q_solution, success): углы суставов и флаг успеха
        """
        q = np.copy(q_seed)
        oMdes = pin.SE3(target_rot, target_pos)

        for i in range(max_iter):
            pin.forwardKinematics(self.model, self.data, q)
            pin.updateFramePlacements(self.model, self.data)

            # Текущая поза TCP
            oMf = self.data.oMf[self.tcp_frame_id]

            # Ошибка в пространстве задач (6D: 3 позиция + 3 ориентация)
            # pin.log возвращает "twist" от oMf до oMdes
            dMi = oMdes.actInv(oMf)
            err = pin.log(dMi).vector      # [6]: [lin_err, ang_err]

            if np.linalg.norm(err) < eps:
                self.get_logger().info(f'IK converged in {i} iterations')
                return q, True

            # Якобиан в локальном фрейме TCP (6 x nq)
            J = pin.computeFrameJacobian(
                self.model, self.data, q, self.tcp_frame_id,
                pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)

            # Damped Least Squares: J^+ = J^T (J J^T + λ²I)^-1
            JJT = J @ J.T
            dq = J.T @ np.linalg.solve(JJT + lam**2 * np.eye(6), err)

            # Интегрируем поправку (Pinocchio учитывает лимиты суставов)
            q = pin.integrate(self.model, q, -dq * 0.5)

            # Клипуем по лимитам суставов (без 7-го — гриппера)
            for j in range(self.model.nq - 1):
                q[j] = np.clip(q[j],
                               self.model.lowerPositionLimit[j],
                               self.model.upperPositionLimit[j])

        self.get_logger().warn(f'IK did NOT converge after {max_iter} iterations')
        return q, False

    # ══════════════════════════════════════════════════════════════════════
    # ДЕТЕКТОР КАСАНИЯ
    # Анализирует скользящее среднее момента на суставах 4,5,6 (запястье)
    # ══════════════════════════════════════════════════════════════════════
    def update_contact_detection(self, tau_measured: np.ndarray):
        """
        Обновляет флаги contact_detected и bottomed_out.
        tau_measured — реальные моменты из joint_states (effort).
        Мониторим суставы запястья (joint4, joint5, joint6).
        """
        # Суммарная нагрузка на "ударные" суставы (индексы 3,4,5)
        contact_force_proxy = np.linalg.norm(tau_measured[3:6])

        self.effort_window.append(contact_force_proxy)
        if len(self.effort_window) > self.WINDOW_SIZE:
            self.effort_window.pop(0)

        mean_effort = np.mean(self.effort_window)

        if mean_effort > self.CONTACT_THRESHOLD and not self.contact_detected:
            self.contact_detected = True
            self.get_logger().info(
                f'CONTACT DETECTED! Mean effort = {mean_effort:.2f} Nm')

        if mean_effort > self.STALL_THRESHOLD and not self.bottomed_out:
            self.bottomed_out = True
            self.get_logger().info(
                f'BUTTON BOTTOMED OUT! Mean effort = {mean_effort:.2f} Nm')

    # ══════════════════════════════════════════════════════════════════════
    # ОСНОВНОЙ CALLBACK — Приём joint_states и вычисление CTC
    # ══════════════════════════════════════════════════════════════════════
    def joint_states_cb(self, msg: JointState):
        name_to_idx = {name: i for i, name in enumerate(msg.name)}
        joint_names = ['joint1', 'joint2', 'joint3',
                       'joint4', 'joint5', 'joint6', 'jointGripper']
        try:
            q  = np.array([msg.position[name_to_idx[n]] for n in joint_names])
            dq = np.array([msg.velocity[name_to_idx[n]] for n in joint_names])
            tau_meas = np.array([msg.effort[name_to_idx[n]] for n in joint_names])
        except KeyError:
            self.get_logger().warn('Joint names mismatch')
            return

        # Сохраняем текущее состояние для использования в control_loop
        self.q_current  = q
        self.dq_current = dq

        # Обновляем детектор касания (только в режиме нажатия)
        if self.state == State.PRESS:
            self.update_contact_detection(tau_meas)

        # ── CTC закон управления ────────────────────────────────────────
        pin.computeAllTerms(self.model, self.data, q, dq)
        M = self.data.M
        C = self.data.C
        g = self.data.g

        if self.state == State.PRESS:
            # В режиме нажатия используем импедансное управление в пространстве задач
            tau = self._impedance_control(q, dq, M, C, g)
        else:
            # Во всех остальных состояниях — стандартный CTC в пространстве суставов
            e   = self.q_d   - q
            de  = self.dq_d  - dq
            tau = M @ (self.ddq_d + self.Kp @ e + self.Kd @ de) + C @ dq + g

        # Гриппер открыт всегда (нажимаем кончиком, не хватаем)
        tau[6] = 0.0

        tau = np.clip(tau, -33.0, 33.0)
        self.pub.publish(Float64MultiArray(data=tau.tolist()))

        if not self.ready:
            self.get_logger().info('CTC controller running')
            self.ready = True

    # ══════════════════════════════════════════════════════════════════════
    # ИМПЕДАНСНОЕ УПРАВЛЕНИЕ В ПРОСТРАНСТВЕ ЗАДАЧ
    # Вычисляет τ через виртуальный импеданс на TCP
    # ══════════════════════════════════════════════════════════════════════
    def _impedance_control(self, q, dq, M, C, g):
        """
        Импедансный закон управления:
        F_imp = K_imp * (x_d - x) + D_imp * (ẋ_d - ẋ)
        τ = J^T * F_imp + g   (с гравитационной компенсацией)
        """
        # Текущая поза и скорость TCP
        pin.forwardKinematics(self.model, self.data, q, dq)
        pin.updateFramePlacements(self.model, self.data)

        oMf = self.data.oMf[self.tcp_frame_id]
        x_cur = oMf.translation          # [3] текущая позиция TCP

        # Линейная скорость TCP через якобиан
        J = pin.computeFrameJacobian(
            self.model, self.data, q, self.tcp_frame_id,
            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        # J.shape = (6, nq); берём только линейную часть (первые 3 строки)
        J_lin = J[:3, :]
        dx_cur = J_lin @ dq[:6]           # [3] линейная скорость TCP

        # Ошибка позиции TCP (движемся от pre_button к button)
        x_err  = self.press_target_pos - x_cur    # [3]
        dx_err = np.zeros(3) - dx_cur             # желаемая скорость = 0

        # Импедансная сила (только линейная часть, 3D)
        F_pos = self.K_imp[:3, :3] @ x_err + self.D_imp[:3, :3] @ dx_err

        # Полная 6D сила (угловая часть = 0)
        F_task = np.concatenate([F_pos, np.zeros(3)])

        # Перевод в пространство суставов через транспонированный якобиан
        tau_imp = J.T @ F_task

        # Добавляем гравитационную компенсацию
        tau = tau_imp + g
        return tau

    # ══════════════════════════════════════════════════════════════════════
    # ГЕНЕРАТОР ТРАЕКТОРИИ (Полином 5-го порядка)
    # ══════════════════════════════════════════════════════════════════════
    def get_spline(self, q_start, q_end, T, t):
        if t <= 0:
            return np.copy(q_start), np.zeros_like(q_start), np.zeros_like(q_start)
        if t >= T:
            return np.copy(q_end), np.zeros_like(q_end), np.zeros_like(q_end)

        tau = t / T
        s   = 10*tau**3 - 15*tau**4 + 6*tau**5
        ds  = (30*tau**2 - 60*tau**3 + 30*tau**4) / T
        dds = (60*tau - 180*tau**2 + 120*tau**3) / (T**2)

        return (q_start + (q_end - q_start)*s,
                (q_end - q_start)*ds,
                (q_end - q_start)*dds)

    # ══════════════════════════════════════════════════════════════════════
    # ЦИКЛ КОНЕЧНОГО АВТОМАТА
    # ══════════════════════════════════════════════════════════════════════
    def control_loop(self):
        if not self.ready:
            return

        self.state_time += self.timer_period

        # ── INIT: Ждём 2 секунды, вычисляем ОЗК для обеих точек ──────────
        if self.state == State.INIT:
            self.q_d, self.dq_d, self.ddq_d = self.get_spline(
                self.q_start, self.q_start, 2.0, self.state_time)

            if self.state_time > 2.0:
                # Решаем ОЗК для точки ПЕРЕД кнопкой
                q_sol, ok = self.solve_ik(
                    self.pre_button_pos, self.button_rot, self.q_ik_seed)
                if ok:
                    self.q_pre_button = q_sol
                    self.transition_to(State.MOVE_TO_PRE)
                else:
                    self.get_logger().error(
                        'IK failed for pre_button! Check button coordinates.')

        # ── MOVE_TO_PRE: Едем по сплайну к точке перед кнопкой ───────────
        elif self.state == State.MOVE_TO_PRE:
            self.q_d, self.dq_d, self.ddq_d = self.get_spline(
                self.q_start, self.q_pre_button, 3.0, self.state_time)
            if self.state_time > 3.5:  # +0.5 сек на стабилизацию
                # Сбрасываем детектор перед нажатием
                self.contact_detected = False
                self.bottomed_out     = False
                self.effort_window    = []
                # Запоминаем целевую точку для импедансного контроллера
                self.press_target_pos = np.copy(self.button_pos)
                self.transition_to(State.PRESS)

        # ── PRESS: Импедансное управление — мягко давим на кнопку ────────
        elif self.state == State.PRESS:
            # q_d здесь не используется (управление через _impedance_control)
            # Ждём сигнала "кнопка нажата до конца"
            if self.bottomed_out:
                self.transition_to(State.HOLD)
            # Таймаут безопасности: если за 5 секунд не нажали — уходим
            elif self.state_time > 5.0:
                self.get_logger().warn('Press timeout! Retracting.')
                self.transition_to(State.RETRACT)

        # ── HOLD: Удерживаем нажатие 0.3 секунды ─────────────────────────
        elif self.state == State.HOLD:
            # Продолжаем импедансное управление
            if self.state_time > 0.3:
                self.transition_to(State.RETRACT)

        # ── RETRACT: Отъезжаем назад к точке pre_button ──────────────────
        elif self.state == State.RETRACT:
            # Используем ПЗК чтобы получить текущую позицию TCP как начало
            # и едем обратно к q_pre_button
            self.q_d, self.dq_d, self.ddq_d = self.get_spline(
                self.q_pre_button, self.q_start, 2.0, self.state_time)
            if self.state_time > 2.0:
                self.transition_to(State.FINISH)

        # ── FINISH: Возврат в начальную позу ─────────────────────────────
        elif self.state == State.FINISH:
            self.q_d, self.dq_d, self.ddq_d = self.get_spline(
                self.q_start, self.q_start, 1.0, self.state_time)
            self.get_logger().info('Task complete!', once=True)

    def transition_to(self, new_state: State):
        self.get_logger().info(
            f'{self.state.name} → {new_state.name}')
        self.state      = new_state
        self.state_time = 0.0


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