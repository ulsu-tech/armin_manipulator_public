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

`cd `ROS_WORKSPACE`/src`

`git clone https://github.com/ulsu-tech/armin_manipulator_public.git`

`cd ..`

`catkin_make`


## Содержание репозитория
Этот git репозиторий содержит следующие ROS-пакеты:
- **armin_description** - пакет, содержащий Xacro (URDF) описание манипулятора и launch файлы для загрузки
модели в RViz
- **armin_moveit_config** - пакет, содержащий конфигурацию, необходимую для MoveIt и файлы инициализации параметров на сервере параметров ROS
- **armin_gazeboed** - пакет, содержащий launch файлы и файлы конфигурации, необходимые для запуска манипулятора в симуляторе Gazebo.
- **armin_manipulators** - пакет, содержащий код приложение, используемых для чтения сигналов с источников управления и формирования команд для манипулятора.
- **armin_base_ikfast_ikfast_plugin** - Inverse Kinematic Solver IKFast собранный для группы планирования base (смотрите пакет armin_moveit_config).

## Запуск в RViz (armin_description)
Для просмотра визуальной модели манипулятора и оценки подвижности суставов, достигаемого пространства,
необходимо выполнить:

`roslaunch armin_description display.launch gui:=true`

Установите необходимые пакеты (к примеру joint_state_publisher и joint_state_publisher_gui) если возникают ошибки при запуске.

После запуска этой команды, будут открыты 2 приложения: окно RViz и окно для установки положения узлов манипулятора. Изображение с примерным видом после запуска команды ниже:
![RViz with armin manipulator](rviz_gui.png "Окно RViz и gui state publisher")

Манипулятор содержит 6 степеней свободы, доступных к установке и захват, фиксирующийся в 2х положениях (открыто и закрыто). Изменение захвата доступно только на физической модели и не описано в URDF модели.

## Запуск симулятора Gazebo и управление из RViz через MoveIt плагин (armin_gazeboed, armin_moveit_config, armin_description)
Симуляция манипулятора возможна в пакете Gazebo.
Для загрузки модели в Gazebo и запуска симулированных приводов и сенсоров, необходимо выполнить:

`roslaunch armin_gazeboed armin_world_remap.launch`

Вызов этой команды откроет окно Gazebo с загруженной моделью манипулятора. Примерный вид как на изображении ниже:
![Gazebo with model loaded](gazebo.png "Окно Gazebo с моделью манипулятора")

При этом менеджер контроллеров загрузил и запустил 2 контроллера:
- /armin/controller/position - контроллер типа FollowJointTrajectory, принимающий на вход траекторию для исполнения, совместимую с генерируемыми MoveIt
- /armin/controller/position/state - контроллер, публикующий состояние узлом манипулятора от симулятора.

Результаты вызовов команд `rosnode list` и `rostopic list` будет похожим на следующий:
![Rostopic and rosnode](nodes_topics.png "Ноды и топики при запущенном Gazebo")

### Запуск RViz и командование из MoveIt
Для планирования траектории и передачи их на исполнение в контроллеры Gazebo, необходимо загрузить
параметры планирования. Для этого выполняем:

`roslaunch armin_moveit_config  move_group.launch`

После чего, можем запустить RViz с настройками инициализированными для работы в MoveIt:

`rosrun rviz rviz  -d $(rospack find armin_gazeboed)/rviz_at_gazebo.rviz`

Если всё прошло хорошо, будет открыто окно RViz с инструментами для настройки планирования движения. Пример того, как это может выглядеть на изображении ниже:
![RViz with MoveIt](rviz_moveit.png "Окно RViz с настроенной группой планирования")


