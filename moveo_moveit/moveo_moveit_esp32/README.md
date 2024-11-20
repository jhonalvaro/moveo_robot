# Proyecto Moveo ROS con ESP32

Este proyecto utiliza ROS junto con las bibliotecas `MultiStepper`, `AccelStepper` y `ServoESP32` para controlar el brazo robótico BCN3D Moveo. En este setup, se utiliza un shield Ramps 1.4 sobre un ESP32.

## Requisitos

- ROS Noetic (o superior) instalado en Ubuntu 20.04
- PlatformIO Core 6.0+ con Visual Studio Code
- ESP32 Dev Module (compatible con ESP32-WROOM-32)
- Shield RAMPS 1.4 para control de motores
- Brazo robótico BCN3D Moveo ensamblado

## Instrucciones de Configuración

### 1. Instalación y Configuración Inicial

```sh
# Instalar dependencias de ROS
sudo apt-get update
sudo apt-get install -y ros-noetic-rosserial-arduino ros-noetic-rosserial

# Crear espacio de trabajo si no existe
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
```

### 2. Clonar el Repositorio

Clona este repositorio en tu máquina local:

```sh
git clone https://github.com/tu_usuario/tu_repositorio.git
cd tu_repositorio
```

### 3. Configurar el Espacio de Trabajo de ROS

Navega al directorio de tu espacio de trabajo de ROS y asegúrate de que los paquetes necesarios estén presentes:

```sh
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 4. Generar las Bibliotecas de `rosserial`

Ejecuta el siguiente comando para generar las bibliotecas necesarias para `rosserial`:

```sh
rosrun rosserial_arduino make_libraries.py ./lib
```

### 5. Configurar PlatformIO

Asegúrate de que tu archivo `platformio.ini` esté configurado correctamente. Aquí tienes un ejemplo:

```ini
[env:esp32]
platform = espressif32
board = esp32dev
framework = arduino
lib_deps =  
  ServoESP32
  AccelStepper
upload_port = /dev/ttyUSB0
monitor_speed = 115200
build_unflags = -std=gnu++11
build_flags = 
  -std=gnu++17
  -I/home/equipo/catkin_ws/devel/include
  -I/home/equipo/catkin_ws/src/moveo_robot/moveo_moveit/moveo_moveit_esp32/lib
  -I/opt/ros/noetic/include
lib_extra_dirs = 
  lib
  /opt/ros/noetic/lib

```

### 6. Compilar el Proyecto

Limpia y compila el proyecto utilizando PlatformIO:

```sh
pio run --target clean
pio run
```

### 7. Cargar el Firmware en el ESP32

Conecta tu ESP32 al puerto USB especificado y carga el firmware:

```sh
pio run --target upload
```

### 8. Monitorear la Salida Serial

Puedes monitorear la salida serial para verificar el funcionamiento del programa:

```sh
pio device monitor
```

## Uso Básico

1. Iniciar el núcleo de ROS:
   ```sh
   roscore
   ```

2. En una nueva terminal, iniciar la comunicación serial con el ESP32:
   ```sh
   # Asegúrate de dar permisos al puerto primero si es necesario
   sudo chmod 666 /dev/ttyUSB0
   rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0 _baud:=115200
   ```

3. Verificar la comunicación:
   ```sh
   # Listar los topics disponibles
   rostopic list

   # Verificar el estado del robot
   rostopic echo /joint_states
   ```

4. Comandos útiles para control:
   ```sh
   # Mover a posición home
   rosservice call /robot/home

   # Enviar comando de movimiento
   rostopic pub /robot/target_position std_msgs/Float32MultiArray "data: [0.0, 0.0, 0.0, 0.0, 0.0]"
   ```

5. Visualización (opcional):
   ```sh
   # Iniciar RViz para visualización 3D
   roslaunch moveo_moveit_config display.launch
   ```

## Comandos de Emergencia

- **Paro de Emergencia**: Presionar `Ctrl+C` en la terminal donde se ejecuta el nodo serial
- **Reset del Sistema**: Presionar el botón de reset en el ESP32
- **Desactivar Motores**: Enviar comando de desactivación
  ```sh
  rostopic pub /robot/disable std_msgs/Empty "{}"
  ```

## Tips de Uso
- Siempre realizar el homing antes de comenzar operaciones
- Verificar que los límites de movimiento estén configurados
- Monitorear la corriente de los motores para evitar sobrecalentamiento
- Mantener el área de trabajo libre de obstáculos

## Indicadores LED
- **LED Verde**: Sistema listo
- **LED Rojo**: Error o paro de emergencia
- **LED Azul parpadeante**: Comunicación activa

## Estructura del Proyecto

- `src/`: Contiene el código fuente del proyecto.
- `lib/`: Contiene las bibliotecas generadas por `rosserial`.
- `platformio.ini`: Archivo de configuración de PlatformIO.

## Diagrama de Conexiones

```ascii
ESP32 ---> RAMPS 1.4
  GPIO21 -> X_STEP
  GPIO22 -> X_DIR
  ...
```

## Solución de Problemas Comunes

1. Error de conexión serial:
   - Verificar permisos: `sudo usermod -a -G dialout $USER`
   - Reconectar ESP32 y reiniciar IDE

2. Errores de compilación:
   - Limpiar proyecto: `pio run -t clean`
   - Actualizar dependencias: `pio lib update`

## Contribuciones

Si deseas contribuir a este proyecto, por favor sigue estos pasos:

1. Haz un fork del repositorio.
2. Crea una nueva rama (`git checkout -b feature/nueva-funcionalidad`).
3. Realiza tus cambios y haz commit (`git commit -am 'Añadir nueva funcionalidad'`).
4. Sube tus cambios a la rama (`git push origin feature/nueva-funcionalidad`).
5. Abre un Pull Request.

## Licencia

Este proyecto está licenciado bajo la Licencia MIT. Consulta el archivo `LICENSE` para más detalles.

## Contacto

Para cualquier pregunta o sugerencia, por favor abre un issue en el repositorio o contacta al autor a través de [tu_email@example.com](mailto:tu_email@example.com).

## Referencias y Recursos

- [Documentación oficial ESP32](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/)
- [Wiki BCN3D Moveo](https://www.bcn3d.com/bcn3d-moveo-the-future-of-learning/)
- [ROS Serial Documentation](http://wiki.ros.org/rosserial)

Este `README.md` proporciona una guía clara y detallada para que otros usuarios puedan configurar y utilizar tu proyecto. Incluye todos los pasos necesarios, desde la clonación del repositorio hasta la carga del firmware en el ESP32 y la monitorización de la salida serial.
