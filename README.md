# mrbobus

4WD skid-steer робот для хакатона «Эвакуация» Кубка РТК. Первый этап — управление ODrive и локализация по Unitree L2.

Базовые исходники импортированы 28.09.2026 с Raspberry Pi `/home/taras/robot/ros2_ws/src`; новые пакеты и изменения ведутся здесь. Это текущая работающая база, не завершённая навигационная система. [Состояние и следующий шаг](docs/status.md).

- `src/robot_base/` — CAN 0.5.6, управление, колёсная одометрия, URDF, веб-пульт и unit-тесты.
- `src/mrbobus_bringup/` — стандартный четырёхколёсный diff_drive_controller, проверенный на mock/vcan, вывешенных колёсах и первом проезде0.5m.
- `src/vendor/` — ограниченная адаптация официального ODrive hardware-плагина под fw0.5.6.
- `src/mrbobus_lio/` — переносимый профиль Unitree L2/Point-LIO; [результаты и ограничения](docs/lio.md).
- `deploy/` — существующие конфиги Pi/Ryzen; пути пока соответствуют размещению на роботе.
- `tools/check_ros.py` — пассивная проверка топиков/TF; `tools/test_vcan.py` — тест на виртуальной CAN-шине Linux.
- `.local/` — локальные необязательные материалы, исключённые из Git.

Проверка логики без ROS и без железа:

```sh
PYTHONPATH=src/robot_base python3 -m unittest discover -s src/robot_base/test -v
```

На Ubuntu 24.04 / ROS 2 Jazzy:

```sh
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select robot_base
colcon test --packages-select robot_base
colcon test-result --verbose
```

Запуск драйвера на CAN может управлять физическим роботом. Развёртывание и испытание движения выполняются отдельно от сборки. На Pi работает новый стандартный драйвер с отдельной кнопкой СТОП, старый сервис остановлен. Кандидат проверен на железе, плавность под нагрузкой ещё не подтверждена; [результаты и команды](docs/status.md).
