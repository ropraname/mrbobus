# Сборка и развёртывание

## Границы воспроизводимости

Репозиторий содержит код и параметры прототипа сентября 2026 года. Требуются
Ubuntu 24.04 / ROS 2 Jazzy, физическая платформа, драйвер Unitree L2, отдельная
сборка Point-LIO, карта для ICP и локальный сервер модели. Зависимости самого
привода собираются по командам README. `mrbobus_console` запускается как Python
модуль из исходников, это не отдельный colcon-пакет.

## Виртуальный CAN

После сборки ROS-пакетов из README на Linux:

```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
PYTHONPATH=src/robot_base python3 tools/test_vcan.py vcan0
```

Интеграционный тест открывает только интерфейс с префиксом `vcan`. Проверки
`tools/test_control_mock.py` и `tools/test_field_navigation.py` требуют ROS и
изолированных доменов; их условия приведены в самих файлах. Они не входят в
быстрый набор GitHub Actions. Сборка C++ и испытания на реальном роботе отдельны
от Python-тестов.

## Аппаратная конфигурация

1. Проверьте CAN ID, направления колёс, Hall CPR, геометрию и стоп-сигнал.
   Соревновательные коэффициенты не универсальны для другой механики.
2. Используйте `src/mrbobus_bringup/launch/control.launch.py`. По умолчанию
   выбран `vcan0`, профиль `bench`, 1 А, а diff-drive контроллер не активирован.
3. До применения сервисов просмотрите `deploy/start-*.sh`, `deploy/*.service`,
   `deploy/cyclone-*.xml`, `src/mrbobus_console/mrbobus_console/server.py` и
   `src/mrbobus_console/config/semantic-map.yaml`: там сохранены пути
   `/home/taras/robot/...` и LAN-адреса конкретного робота.
4. Для LIO следуйте [docs/lio.md](lio.md). Скрипт `tools/setup_lio.sh` закрепляет
   upstream commit и применяет patch, но не устанавливает драйвер L2.
5. Для модели см. [deploy/inference](../deploy/inference/README.md).
   Веса, runtime и секреты хранятся вне репозитория.
6. Пульт: [ELRS/CRSF](elrs.md). В прошедшем соревнование профиле потеря пульта
   переключает источник на AUTO по требованию команды; это надо учитывать при
   переносе робота в другую среду.

## Панель

На настроенном роботе с активными ROS workspace:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source /home/taras/robot/lio_ws/install/setup.bash
export ROS_DOMAIN_ID=42
export PYTHONPATH="$PWD/src/mrbobus_console:${PYTHONPATH:-}"
python3 -m mrbobus_console.server --address 192.168.67.149
```

Адрес должен принадлежать машине; замените его при другой сети. Панель по адресу
`http://192.168.67.149:8080/`, сценарий — `/mission`. Это доверенная локальная
сеть, а не готовый защищённый интернет-сервис. Запуск панели сам запускает
камеру и LIO; привод включается отдельным действием при выполненных проверках.
Сервисы требуют настроенных прав пользователя на видео, CAN и необходимые
операции systemd. Не копируйте sudo-права без просмотра команд.

## Карта

```bash
python3 tools/build_navigation_map.py \
  src/mrbobus_console/config/semantic-map.yaml /tmp/mrbobus-navigation-map
```

Команда генерирует PGM/YAML статического слоя, а не облако локализации.
Здания и границы непроходимы; газон вокруг башни имеет конечную повышенную
стоимость; настилы мостов открыты, перила остаются препятствиями.
Проходимость требует проверки геометрии и текущих данных лидара.

## Что не делает CI

CI не проверяет электропитание, CAN, радио, USB, ROS TF, реальное движение,
качество локализации и полноту миссии. Успех CI означает только прохождение
включённых автономных Python-тестов.
