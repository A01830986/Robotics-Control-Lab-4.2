# Robotics-Control-Lab-4.2
# Control de Posición del xArm Lite 6 bajo Perturbaciones

## 📌 Introducción

Este repositorio contiene el código y el análisis para el diseño e implementación de un controlador de posición en el espacio cartesiano para el robot **xArm Lite 6** utilizando **MoveIt Servo**.

El proyecto evalúa la robustez de un controlador PID diseñado para seguir una trayectoria en forma de ocho (Lemniscata de Bernoulli) bajo condiciones de perturbación deterministas (senoidales) y estocásticas (gaussianas).

## ⚙️ Arquitectura del Sistema

El sistema opera mediante una cadena de nodos en ROS 2 Humble:

1. **Generador de Trayectorias:** Publica la pose deseada (Lemniscata) basándose en parámetros predefinidos y un algoritmo de *soft-start*.
2. **Controlador PID:** Calcula el error cartesiano en tiempo real leyendo el árbol de transformadas (TF) desde `link_base` hasta `link_eef`.
3. **Inyector de Perturbaciones:** Introduce ruido controlado (senoidal o gaussiano) directamente en la señal de control.
4. **MoveIt Servo:** Recibe comandos `TwistStamped` y ejecuta el movimiento de forma fluida en el hardware físico.

## 🛠️ Características del Controlador

Se implementó un controlador PID para maximizar la corrección de errores durante el seguimiento de la trayectoria:

* **Ganancias Sintonizadas:** `Kp = 3.5`, `Kd = 0.2`, `Ki = 1.0`
* **Restricciones de Seguridad:** * **Zona muerta (Deadband):** `0.002 m` para ignorar ruidos menores provenientes de las lecturas de TF.
* **Saturación de Velocidad:** Límite estricto de `0.12 m/s` por eje para garantizar un funcionamiento seguro del hardware.



## 📊 Evaluación y Resultados Cuantitativos

Las pruebas experimentales se realizaron manteniendo constantes las ganancias del controlador. Se calculó el Root Mean Square Error (RMSE) tridimensional para el caso base y el caso perturbado con PID:

| Métrica | Recorrido NORMAL (Baseline) | Recorrido CON PID (Perturbado) |
| --- | --- | --- |
| **RMSE Eje X** | 0.024172 m | 0.219357 m |
| **RMSE Eje Y** | 0.016826 m | 0.018044 m |
| **RMSE Eje Z** | 0.003725 m | 0.330646 m |
| **RMSE Total** | 0.029686 m | 0.397202 m |
| **Error Máx. Absoluto** | 0.064831 m | 0.427001 m |

### Gráficas de Rendimiento

* **Posición Deseada vs. Real (Plano X-Y):**
* **Magnitud del Error a lo largo del Tiempo:**
* **Magnitud de la Velocidad Comandada:**

## 🔬 Análisis Avanzado y Conclusiones

* **Desfase Temporal vs. Error Espacial (Tracking Lag):** Aunque las métricas numéricas muestran un RMSE de ~0.21 m en el eje X para el recorrido con perturbaciones, las gráficas del plano X-Y demuestran que el robot mantuvo la trayectoria geométrica de manera excepcional. La perturbación no generó un desvío espacial (*cross-track error*), sino un retraso temporal en el seguimiento (*tracking lag*). Las fórmulas de distancia punto-a-punto penalizan severamente este desfase.
* **Efecto del Windup (Término Integral):** El aumento masivo del error en el eje Z (0.33 m) ilustra el fenómeno de *integral windup*. Al enfrentarse a una perturbación constante, el término `Ki = 1.0` acumuló error rápidamente, empujando al Efector Final fuera del plano de trabajo vertical. Esto demuestra la necesidad crítica de implementar límites estrictos (*anti-windup clamping*) para entornos perturbados.
* **Sensibilidad en el Dominio de la Frecuencia:** El sistema compensa exitosamente perturbaciones de baja frecuencia (senoidales). Sin embargo, el ruido de alta frecuencia (Gaussiano) excita el término derivativo (`Kd = 0.2`), el cual actúa como un filtro pasa-altas. Esto amplifica el ruido en la señal de salida, induciendo comandos de velocidad oscilatorios y revelando el compromiso empírico necesario entre la ganancia derivativa y el margen de estabilidad.

## 🚀 Cómo ejecutar (Usage)

Para reproducir los resultados en el entorno real o simulado de ROS 2:

1. **Lanzar la interfaz de MoveIt Servo (Hardware Real):**
```bash
ros2 launch xarm_moveit_servo lite6_moveit_servo_realmove.launch.py robot_ip:=192.168.1.123

```


2. **Ejecutar el Controlador PID y Generador de Trayectoria:**
```bash
ros2 run xarm_perturbations circle_maker

```


3. **Inyectar Perturbaciones (Ejemplo: Senoidal):**
```bash
ros2 run xarm_perturbations perturbation_injector --ros-args -p output_topic:=/servo_server/delta_twist_cmds -p pub_reliability:=reliable -p enabled:=true -p mode:=sine -p sine_freq_hz:=1.0 -p sine_amp_linear:=0.01 -p sine_axis:=x -p base_linear:="[0.02, -0.02, 0.02]" -p debug:=true

```
