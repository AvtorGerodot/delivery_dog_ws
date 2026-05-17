# delivery_dog_bringup

Composite-пакет для совместного запуска в Gazebo Harmonic + ROS2 Jazzy:

- `mobile_z1` - мобильная mecanum-платформа с прикреплённым на крышу
  манипулятором Z1 (временная замена квадрупеду Unitree B2).
- `entrance_group_v3` - модель входной группы (стена + дверь с доводчиком
  + домофон с кнопками).

Всё запускается в **одном** инстансе Gazebo с **одним** `controller_manager`
и **одним** ROS-GZ мостом.

## Состав пакета

```
delivery_dog_bringup/
├── package.xml
├── CMakeLists.txt
├── urdf/
│   └── mobile_z1.urdf.xacro          # mobile_base + Z1 в одной модели
├── worlds/
│   └── delivery_world.sdf            # общий мир (земля, освещение, физика)
├── config/
│   ├── mobile_z1_controllers.yaml    # один controller_manager на оба ros2_control
│   └── ros_gz_bridge.yaml            # общий мост
├── launch/
│   └── full_scene.launch.py          # главный launch-файл
├── scripts/
│   └── mecanum_kinematics_node.py    # /cmd_vel -> 4 колесные скорости
└── README.md
```

## Зависимости и сборка

В рабочем пространстве (`~/delivery_dog_ws`) должны лежать пакеты:

- `z1_model` (с НОВЫМИ файлами `urdf/z1_arm_macro.urdf.xacro` и
  `urdf/z1_standalone.urdf.xacro`).
- `mobile_base_model` (новый, в этом архиве).
- `entrance_group_v3` (без изменений, рабочая версия с textures).
- `dynamic_control_py` (без изменений).

Сборка:

```bash
cd ~/delivery_dog_ws
colcon build --symlink-install
source install/setup.bash
```

## Запуск всей сцены

```bash
ros2 launch delivery_dog_bringup full_scene.launch.py
```

Опции:

```bash
# Подвинуть подъезд
ros2 launch delivery_dog_bringup full_scene.launch.py entrance_x:=3.0 entrance_y:=0.5

# Стартовая позиция робота
ros2 launch delivery_dog_bringup full_scene.launch.py robot_x:=-1.0

# С RViz
ros2 launch delivery_dog_bringup full_scene.launch.py rviz:=true
```

Что должно произойти:

1. Открывается Gazebo Harmonic с миром `delivery_world.sdf`.
2. Спавнится `mobile_z1` (платформа с Z1 на крышке) у (0,0).
3. Спавнится `entrance_group_v3` на (2.5, 0) с yaw=π.
4. Поднимаются 3 контроллера: `joint_state_broadcaster`, `effort_controller`,
   `wheels_velocity_controller` - все в namespace `/robot`.
5. Запускается `mecanum_kinematics_node`, конвертирующий `/cmd_vel` в команды
   на 4 колеса.
6. Запускается мост: время, `/cmd_vel`, joint_states подъезда.

## Как двигать платформу

```bash
# Линейно вперёд
ros2 topic pub --once /cmd_vel geometry_msgs/Twist '{linear: {x: 0.3}}'

# Боком (фишка mecanum!)
ros2 topic pub --once /cmd_vel geometry_msgs/Twist '{linear: {y: 0.3}}'

# Разворот на месте
ros2 topic pub --once /cmd_vel geometry_msgs/Twist '{angular: {z: 0.5}}'

# Стоп
ros2 topic pub --once /cmd_vel geometry_msgs/Twist '{}'
```

## Как двигать Z1

Без изменений: dynamic_control_py теперь работает в namespace `/robot`.
Поэтому путь команд изменился:

| Старый топик                              | Новый топик                          |
|-------------------------------------------|--------------------------------------|
| `/z1/effort_controller/commands`          | `/robot/effort_controller/commands`  |
| `/z1/controller_manager/...`              | `/robot/controller_manager/...`      |
| `/joint_states`                           | `/joint_states` (в `/robot/joint_states` после spawnera) |

Открой `dynamic_control_py/dynamic_control_node.py` и поменяй namespace
`/z1` на `/robot`, либо запускай узел так:

```bash
ros2 run dynamic_control_py dynamics_control_node \
    --ros-args -r __ns:=/robot
```

## Архитектурные решения

### 1. Один controller_manager

`gz_ros2_control` плагин **один на всю сцену** и подхватывает оба
`<ros2_control>` блока (один - mobile_base, другой - Z1). Так избегаем
двух менеджеров, конкурирующих за `/joint_states`.

### 2. Mecanum-кинематика - в отдельном узле

В ros2_controllers в Jazzy нет стабильного `mecanum_drive_controller`,
поэтому делаем сами: на колёса даём `velocity_controllers/JointGroupVelocityController`,
а преобразование `cmd_vel -> 4*velocity` - в отдельном Python-узле.
Это даёт гибкость: легко добавить odom-publisher, slip-модель и т.п.

### 3. Z1 через макрос

В пакете `z1_model` есть НОВЫЙ файл `z1_arm_macro.urdf.xacro` рядом со
старым `z1_model.urdf.xacro`. Старый запуск (`ros2 launch z1_model
z1_model.launch.py`) продолжает работать без изменений. Composite-пакет
использует макрос, чтобы прикрепить руку к произвольному parent-link
с произвольным prefix.

Когда дойдёт до настоящего Unitree B2, ты просто заменишь
`xacro:mobile_base` на `xacro:b2_robot` в `mobile_z1.urdf.xacro`, и больше
ничего трогать не придётся.

### 4. GZ_SIM_RESOURCE_PATH

Launch-файл добавляет родителей share-папок всех 3 пакетов в
`GZ_SIM_RESOURCE_PATH`, чтобы Gazebo разрешал
`model://entrance_group_v3/materials/textures/...`.

### 5. Спавн контроллеров через RegisterEventHandler

Контроллеры запускаются только после того, как `ros_gz_sim::create`
завершит работу (модель уже создана и controller_manager поднялся).
Без этого спавнер падает с ошибкой "controller_manager not available".

## Тонкие места

- **Скользит/буксует**: подкрути `<mu1>/<mu2>` для колёс в
  `mobile_base_model/urdf/mobile_base_macro.urdf.xacro`.
- **Не нашлись текстуры подъезда**: проверь, что
  `echo $GZ_SIM_RESOURCE_PATH | tr ':' '\n' | grep entrance_group_v3` показывает
  родителя share-папки.
- **/robot/joint_states пустой**: убедись что `joint_state_broadcaster`
  активен - `ros2 control list_controllers --controller-manager /robot/controller_manager`.
- **dynamic_control_node не работает после миграции**: поменяй namespace
  или префикс топика с `/z1/...` на `/robot/...`.
