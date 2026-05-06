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
    INIT = 0
    APPROACH = 1
    DESCEND = 2
    GRASP = 3
    LIFT = 4
    MOVE = 5
    RELEASE = 6   
    DETACH = 7    
    RETURN = 8    
    FINISH = 9    

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
        self.Kp = np.diag([300.0, 400.0, 400.0, 2000.0, 2000.0, 2000.0, 50])
        self.Kd = np.diag([ 20.0,  20.0,  30.0,  30.0,  30.0,  30.0, 5])

        # Целевая конфигурация (рад) — начальная поза 
        self.q_d   = np.array([0.0, 0.0, -0.035, 0.0, 0.0, 0.0, 0.0]) # -1.047 # ([1.57, 2.007, -1.833, -1.0, 0.0, 0.0, 0.0]) 
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

         # Начальная поза (из xacro)
        self.q_start = np.array([0.0, 0.0, -0.06, 0.0, 0.0, 0.0, 0.0]) #([0.0, 0.0, -0.06, 0.0, 0.0, 0.0, 0.0])
        # Точка подхода: над кубиком (y=0.5, z=0.15)
        self.q_approach = np.array([1.57, 2.007, -1.833, 1.326, 0.0, 0.0, 0.0])
        # Точка захвата: опустились к кубику (z=0.025)
        self.q_grasp = np.array([1.57, 2.06, -1.36, 0.78, 0.0, 0.0, 0.0])
        # Точка назначения (например, перенести на x=0.5, y=0)
        self.q_target = np.array([0.0, 2.007, -1.833, 1.326, 0.0, 0.0, 0.0])
        # Точка назначения (например, перенести на x=0.5, y=0)
        self.q_target = np.array([0.0, 2.007, -1.833, 1.326, 0.0, 0.0, 0.0])
        # Точка, в которой мы отпускаем кубик (опускаем руку в целевой позиции)
        self.q_release = np.array([0.0, 2.06, -1.36, 0.78, 0.0, 0.0, 0.0])

        # Переменные конечного автомата
        self.state = State.INIT
        self.state_time = 0.0     # Время с начала текущего состояния
        self.timer_period = 0.01  # 100 Гц
        
        # Запускаем таймер, который будет управлять стейт-машиной
        self.timer = self.create_timer(self.timer_period, self.control_loop)


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

        # print(tau[6])
        # Специальная логика для гриппера (7-й сустав)
        if self.state in [State.GRASP, State.LIFT, State.MOVE, State.RELEASE]:
            # Давим константным моментом чтобы зажать кубик
            tau[6] = 20.0 
        else:
            # Держим гриппер открытым
            tau[6] = -20.0 

        # защита от выхода за пределы +-33 Н·м
        tau = np.clip(tau, -33.0, 33.0)

        cmd = Float64MultiArray(data=tau.tolist())
        self.pub.publish(cmd)

        if not self.ready:
            self.get_logger().info('CTC controller running')
            self.ready = True


    def get_spline(self, q_start, q_end, T, t):
        """Возвращает q, dq, ddq по полиному 5-го порядка."""
        if t <= 0:
            return np.copy(q_start), np.zeros_like(q_start), np.zeros_like(q_start)
        if t >= T:
            return np.copy(q_end), np.zeros_like(q_end), np.zeros_like(q_end)

        tau = t / T
        s   = 10 * tau**3 - 15 * tau**4 + 6 * tau**5
        ds  = (30 * tau**2 - 60 * tau**3 + 30 * tau**4) / T
        dds = (60 * tau - 180 * tau**2 + 120 * tau**3) / (T**2)

        q   = q_start + (q_end - q_start) * s
        dq  = (q_end - q_start) * ds
        ddq = (q_end - q_start) * dds

        return q, dq, ddq
    

    def control_loop(self):
        if not self.ready:
            return  # Ждём первые данные от joint_states

        self.state_time += self.timer_period

        if self.state == State.INIT:
            # Удерживаем начальную позу 2 секунды
            self.q_d, self.dq_d, self.ddq_d = self.get_spline(self.q_start, self.q_start, 2.0, self.state_time)
            if self.state_time > 2.0:
                self.transition_to(State.APPROACH)

        elif self.state == State.APPROACH:
            # Движемся к точке над кубиком за 3 секунды
            self.q_d, self.dq_d, self.ddq_d = self.get_spline(self.q_start, self.q_approach, 4.0, self.state_time)
            if self.state_time > 3.0:
                self.transition_to(State.DESCEND)

        elif self.state == State.DESCEND:
            # Опускаемся на кубик за 2 секунды
            self.q_d, self.dq_d, self.ddq_d = self.get_spline(self.q_approach, self.q_grasp, 2.0, self.state_time)
            if self.state_time > 2.0:
                self.transition_to(State.GRASP)

        elif self.state == State.GRASP:
            # Стоим на месте, зажимаем гриппер (отдельно обработаем момент в joint_states_cb)
            # Ждём 1 секунду на захват, потом обновляем модель
            self.q_d, self.dq_d, self.ddq_d = self.get_spline(self.q_grasp, self.q_grasp, 2.0, self.state_time)
            
            if self.state_time > 1.0:
                self.attach_cube_to_model()
                self.transition_to(State.LIFT)

        elif self.state == State.LIFT:
            # Поднимаем кубик обратно в позу approach за 2 секунды
            self.q_d, self.dq_d, self.ddq_d = self.get_spline(self.q_grasp, self.q_approach, 2.0, self.state_time)
            if self.state_time > 2.0:
                self.transition_to(State.MOVE)

        elif self.state == State.MOVE:
            self.q_d, self.dq_d, self.ddq_d = self.get_spline(self.q_approach, self.q_target, 3.0, self.state_time)
            if self.state_time > 3.0:
                self.transition_to(State.RELEASE)

        elif self.state == State.RELEASE:
            # Опускаем руку вниз КУБИК ВСЁ ЕЩЁ ЗАЖАТ
            self.q_d, self.dq_d, self.ddq_d = self.get_spline(self.q_target, self.q_release, 2.0, self.state_time)
            if self.state_time > 2.0:
                self.transition_to(State.DETACH)

        elif self.state == State.DETACH:
            # Стоим на месте, гриппер открывается (см. joint_states_cb)
            self.q_d, self.dq_d, self.ddq_d = self.get_spline(self.q_release, self.q_release, 1.0, self.state_time)
            
            # Ждём 1 секунду, пока гриппер механически разожмётся, затем "отвязываем" массу
            if self.state_time > 1.0:
                self.detach_cube_from_model()
                self.transition_to(State.RETURN)

        elif self.state == State.RETURN:
            # Возвращаемся в начальную позицию с пустым гриппером
            self.q_d, self.dq_d, self.ddq_d = self.get_spline(self.q_release, self.q_start, 4.0, self.state_time)
            if self.state_time > 4.0:
                self.transition_to(State.FINISH)
                
        elif self.state == State.FINISH:
            # Удерживаем начальную позицию бесконечно
            self.q_d, self.dq_d, self.ddq_d = self.get_spline(self.q_start, self.q_start, 1.0, self.state_time)

    def transition_to(self, new_state):
        self.get_logger().info(f"Transitioning to {new_state.name}")
        self.state = new_state
        self.state_time = 0.0


    def attach_cube_to_model(self):
        self.get_logger().info("Attaching 0.5kg cube to dynamic model!")
        
        # Масса кубика 0.5 кг
        mass = 0.5
        # Смещение центра масс (в локальных координатах 6-го звена)
        # Зависит от длины гриппера. Допустим, кубик зажат на расстоянии 5 см (0.05 м) по оси X.
        com = np.array([0.05, 0.0, 0.0])
        # Моменты инерции из SDF
        I_c = np.diag([2.0833e-4, 2.0833e-4, 2.0833e-4])
        
        inertia_cube = pin.Inertia(mass, com, I_c)
        
        # ID звена, к которому крепим (в Pinocchio имя совпадает с URDF)
        joint_id = self.model.getJointId('joint6') # Убедитесь, что имя совпадает с URDF
        
        # Добавляем массу к модели
        self.model.appendBodyToJoint(joint_id, inertia_cube, pin.SE3.Identity())
        
        # ПЕРЕСОЗДАЁМ data, так как модель изменилась!
        self.data = self.model.createData()

    def detach_cube_from_model(self):
        self.get_logger().info("Detaching cube (restoring original dynamics)")
        
        # Пересоздаём оригинальную модель из URDF, чтобы сбросить прикреплённую массу
        z1_pkg_share = get_package_share_directory('z1_model')
        urdf_path = os.path.join(z1_pkg_share, 'urdf', 'z1_preview.urdf')
        
        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data  = self.model.createData()





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