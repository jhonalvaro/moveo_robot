# moveo_ros
Paquetes ROS que se pueden usar para planificar y ejecutar trayectorias de movimiento para el brazo robótico BCN3D Moveo en simulación y en la vida real.
### [¡Demostración en Video Aquí!](https://youtu.be/2RcTTqs17O8)

- **_Nueva Funcionalidad_: Recogida y Colocación Específica de Objetos** (Con una cámara web ordinaria, Tensorflow, OpenCV y ROS, puedes 'recoger y colocar' (o clasificar) objetos que se detectan en tiempo real)
	- **[Demostración en Video](https://youtu.be/kkUbyFa2MWc)**
	- **[Cómo Usar](https://github.com/jesseweisberg/moveo_ros/tree/master/moveo_moveit/scripts)**

## Cómo Usar:

### Configuración de la Simulación BCN3D con Planificación de Movimiento
![moveit_screenshot.png](/moveit_screenshot.png)

1. Asegúrate de tener ROS instalado correctamente con un espacio de trabajo funcional. Yo usé ROS Kinetic en Ubuntu 16.04 (si tienes una distribución diferente, es posible que necesites cambiar algunas cosas). Actualmente tengo 'moveo_ros' en la carpeta 'src' de mi espacio de trabajo catkin.

2. Para planificar y ejecutar trayectorias para el Moveo en simulación (RVIZ con el plugin Moveit), ejecuta el siguiente comando en la terminal:
	