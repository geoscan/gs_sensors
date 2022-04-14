# Описание пакета gs_sensors

## Описание:
В данном пакете представлены инструменты для получение данных с сенсоров

## Состав пакета:
Классы:
* SensorManager

Ноды:
* ultrasonic_node.py

## Описание классов:

### 1. SensorManager
Класс менеджера сенсоров

#### Инициализация:
Без параметров

#### Поля:
* error_number - float
* __battery_state - gs_interfaces.msg.SimpleBatteryState
* __gyro - geometry_msgs.msg.Point
* __accel - geometry_msgs.msg.Point
* __orientation - gs_interfaces.msg.Orientation
* __altitude - float
* __mag - geometry_msgs.msg.Point
* __alive - rospy.ServiceProxy: gs_interfaces.srv.Live
* __gyro_subscriber - rospy.Subscriber: geometry_msgs.msg.Point
* __accel_subscriber - rospy.Subscriber: geometry_msgs.msg.Point
* __orientation_subscriber - rospy.Subscriber: gs_interfaces.msg.Orientation
* __altitude_subscriber - rospy.Subscriber: std_msgs.msg.Float32
* __mag_subscriber - rospy.Subscriber: geometry_msgs.msg.Point
* __power_subscriber - rospy.Subscriber: gs_interfaces.msg.SimpleBatteryState

#### Методы:
* gyro - возвращает данные c гироскопа (gx,gy,gz)
* accel -  возвращает данные c акселерометра (ax,ay,az)
* orientation - возвращает данные положения (roll,pitch,azimuth)
* altitude - возвращает данные высоты по барометру
* mag - возвращает данные с магнитометра
* power - возвращает заряд АКБ (charge, sec)

#### Используемые сервисы:
* geoscan/alive (gs_interfaces/Live)

#### Используемые топики:
* geoscan/sensors/gyro (geometry_msgs/Point)
* geoscan/sensors/accel (geometry_msgs/Point)
* geoscan/sensors/orientation (gs_interfaces/Orientation)
* geoscan/sensors/altitude (std_msgs/Float32)
* geoscan/sensors/mag (geometry_msgs/Point)
* geoscan/battery_state (gs_interfaces/SimpleBatteryState)

## Описание нод:

### 1. ultrasonic_node
Нода ультрозвукового датчика HC-SR04

#### Параметры:
* trig - номер GPIO порта, соответствующий TRIG
* echo - номер GPIO порта, соответствующий ECHO

#### Топики
* ultrasonic_sensor/trig_<номер TRIG порта>_echo\_<номер ECHO порта> (std_msgs/Float32)

## Необходимые пакеты:
ROS:
* gs_interfaces
* gs_core
* std_msgs
* geometry_msgs

## Примечание:
Все классы в данном пакете могут быть использованы только при запущеной ноде ros_plaz_node.py из пакета gs_core
