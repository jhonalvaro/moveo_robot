# Proyecto Moveo ROS con ESP32

Este proyecto utiliza ROS junto con las bibliotecas `MultiStepper`, `AccelStepper` y `ServoESP32` para controlar el brazo robótico BCN3D Moveo. En este setup, se utiliza un shield Ramps 1.4 sobre un ESP32.

## Requisitos

- ROS (Robot Operating System) instalado en tu sistema.
- PlatformIO instalado en tu editor de código (recomendado Visual Studio Code).
- Un ESP32 Dev Module.
- Un shield Ramps 1.4.

## Instrucciones de Configuración

### 1. Clonar el Repositorio

Clona este repositorio en tu máquina local:

```sh
git clone https://github.com/tu_usuario/tu_repositorio.git
cd tu_repositorio
```

### 2. Configurar el Espacio de Trabajo de ROS

Navega al directorio de tu espacio de trabajo de ROS y asegúrate de que los paquetes necesarios estén presentes:

```sh
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 3. Generar las Bibliotecas de `rosserial`

Ejecuta el siguiente comando para generar las bibliotecas necesarias para `rosserial`:

```sh
rosrun rosserial_arduino make_libraries.py ./lib
```

### 4. Configurar PlatformIO

Asegúrate de que tu archivo `platformio.ini` esté configurado correctamente. Aquí tienes un ejemplo:

```ini
[env:esp32]
platform = espressif32
board = esp32dev
framework = arduino
lib_deps =
  Rosserial Arduino Library
  ServoESP32
  AccelStepper
upload_port = /dev/ttyUSB0
monitor_speed = 115200
build_unflags = -std=gnu++11
build_flags = -std=gnu++17
lib_extra_dirs = lib/ros_lib
```

### 5. Compilar el Proyecto

Limpia y compila el proyecto utilizando PlatformIO:

```sh
pio run --target clean
pio run
```

### 6. Cargar el Firmware en el ESP32

Conecta tu ESP32 al puerto USB especificado y carga el firmware:

```sh
pio run --target upload
```

### 7. Monitorear la Salida Serial

Puedes monitorear la salida serial para verificar el funcionamiento del programa:

```sh
pio device monitor
```

## Estructura del Proyecto

- `src/`: Contiene el código fuente del proyecto.
- `lib/`: Contiene las bibliotecas generadas por `rosserial`.
- `platformio.ini`: Archivo de configuración de PlatformIO.

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

Este `README.md` proporciona una guía clara y detallada para que otros usuarios puedan configurar y utilizar tu proyecto. Incluye todos los pasos necesarios, desde la clonación del repositorio hasta la carga del firmware en el ESP32 y la monitorización de la salida serial.
