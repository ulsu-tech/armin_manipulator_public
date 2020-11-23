# armin_manipulator_public

This public repo contains model and required ROS-launch files
to let interested people in getting familiar with solution offered
by NITI UlSU.

If you are interested in English version of this file, please follow this
link (not done yet).

Этот публичный репозиторий содержит ROS-пакеты, необходимые заинтересованным
лицам, для ознакомления с моделью и запуска модели манипулятора в симуляторе Gazebo.

Манипулятор разработан в рамках работы над созданием радиационно-защищённого манипулятора,
призванного заменить используемые в настоящее время механические копирующие манипуляторы.

Версия проверена на ROS Melodic. Частичная совместимость могла сохраниться с ROS Kinetic.

## Получение актуальной версии
Для запуска команд, описанных ниже, необходимо в вашем рабочем окружении ROS выполнить следующее:
- cd `ROS_WORKSPACE`/src
- git clone https://github.com/ulsu-tech/armin_manipulator_public.git
- cd ..
- catkin_make


## Содержание репозитория
Этот git репозиторий содержит следующие ROS-пакеты:
- armin_description - пакет, содержащий Xacro (URDF) описание манипулятора и launch файлы для загрузки
модели в RViz
- armin_moveit_config - пакет, содержащий конфигурацию, необходимую для MoveIt и файлы инициализации параметров на сервере параметров ROS
- armin_gazeboed - пакет, содержащий launch файлы и файлы конфигурации, необходимые для запуска манипулятора в симуляторе Gazebo.
- armin_manipulators - пакет, содержащий код приложение, используемых для чтения сигналов с источников управления и формирования команд для манипулятора.
- armin_base_ikfast_ikfast_plugin - Inverse Kinematic Solver IKFast собранный для группы планирования base (смотрите пакет armin_moveit_config).

## Запуск в RViz
Для просмотра визуальной модели манипулятора и оценки подвижности суставов, достигаемого пространства,
необходимо выполнить:
roslaunch armin_description display.launch gui:=true

Установите необходимые пакеты (к примеру joint_state_publisher и joint_state_publisher_gui) если возникают ошибки при запуске.

После запуска этой команды, будут открыты 2 приложения: окно RViz и окно для установки положения узлов манипулятора.

Манипулятор содержит 6 степеней свободы, доступных к установке и захват, фиксирующийся в 2х положениях (открыто и закрыто). Изменение захвата доступно только на физической модели и не описано в URDF модели.
