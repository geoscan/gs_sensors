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
* __alive - rospy.ServiceProxy: gs_interfaces.srv.Live
* __lpsvel_service - rospy.ServiceProxy: gs_interfaces.srv.LpsVel
* __lpsyaw_service - rospy.ServiceProxy: gs_interfaces.srv.LpsYaw
* __gyro_service - rospy.ServiceProxy: gs_interfaces.srv.Gyro
* __accel_service - rospy.ServiceProxy: gs_interfaces.srv.Accel
* __orientation_service - rospy.ServiceProxy: gs_interfaces.srv.Orientation
* __altitude_service - rospy.ServiceProxy: gs_interfaces.srv.Altitude

#### Методы:
* lpsVelocity - возвращает скорость коптера возвращаемую LPS (vx,vy,vz)
* lpsYaw - возвращает угол поворота в системе LPS
* gyro - возвращает данные c гироскопа (gx,gy,gz)
* accel -  возвращает данные c акселерометра (ax,ay,az)
* orientation - возвращает данные положения (roll,pitch,azimuth)
* altitude - возвращает данные высоты по барометру

#### Используемые сервисы:
* geoscan/alive (gs_interfaces/Live)
* geoscan/sensors/lpsvel_service (gs_interfaces/LpsVel)
* geoscan/sensors/lpsyaw_service (gs_interfaces/LpsYaw)
* geoscan/sensors/gyro_service (gs_interfaces/Gyro)
* geoscan/sensors/accel_service (gs_interfaces/Accel)
* geoscan/sensors/orientation_service (gs_interfaces/Orientation)
* geoscan/sensors/altitude_service (gs_interfaces/Altitude)

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

## Примечание:
Все классы в данном пакете могут быть использованы только при запущеной ноде ros_serial_node.py из пакета gs_core
