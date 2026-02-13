# 📝 Workflow de Grabación de Waypoints

## 📍 Ubicación de Archivos (ACTUALIZADO)

- **Waypoints guardados en**: `autonomous_patrol/share/data/waypoints.yaml`
  - Ruta absoluta: `/home/ferradar/ros2_ws/install/autonomous_patrol/share/autonomous_patrol/data/waypoints.yaml`
  - Se crea automáticamente si no existe

- **Resultados guardados en**: `autonomous_patrol/share/results/metrics.json`
  - Ruta absoluta: `/home/ferradar/ros2_ws/install/autonomous_patrol/share/autonomous_patrol/results/metrics.json`

## ⚙️ Configuración (en `autonomous_patrol_config.yaml`)

### Opciones de Muestreo

```yaml
recorder:
  sampling_mode: "distance"                    # OPCIÓN 1: "distance" o "frequency"
  min_distance_between_waypoints: 0.1          # Si distance: metros entre puntos
  sampling_frequency: 5.0                      # Si frequency: Hz
```

**Ejemplo 1**: Grabar cada 0.1m (default - recomendado)
```yaml
sampling_mode: "distance"
min_distance_between_waypoints: 0.1
```

**Ejemplo 2**: Grabar cada 0.05m (más puntos, más detalle)
```yaml
sampling_mode: "distance"
min_distance_between_waypoints: 0.05
```

**Ejemplo 3**: Grabar a 10 Hz (independiente de velocidad)
```yaml
sampling_mode: "frequency"
sampling_frequency: 10.0
```

## 🚀 Paso a Paso: Grabación Correcta

### Terminal 1: Lanzar Gazebo
```bash
cd ~/ros2_ws && source install/setup.bash
ros2 launch robot_description robot.launch.py
```
*(Espera a que Gazebo y RViz se abran)*

### Terminal 2: Grabar Waypoints  
```bash
cd ~/ros2_ws && source install/setup.bash
ros2 launch autonomous_patrol record_waypoints.launch.py
```
*(Se mostrará: "Waypoint Recorder initialized")*

### Terminal 3: Teleop
```bash
cd ~/ros2_ws && source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Ahora en Terminal 3: MUEVE EL ROBOT FORMANDO UNA RUTA

**IMPORTANTE**: Forma una ruta interesante, NO solo una línea recta:

- Opción A: Rectángulo
  - Adelante 1m (comando: `w w w`)
  - Girar derecha 90° (comando: `a` o `d`)
  - Adelante 1m
  - Girar 90°
  - Adelante 1m
  - Girar 90°
  - Adelante 1m (vuelves al inicio)

- Opción B: Círculo
  - Mantén presionado `w` y `d` simultáneamente unos segundos

- Opción C: Zigzag
  - Adelante, gira un poco, adelante, gira otro lado, etc.

### Detener Grabación: `Ctrl+C` en Terminal 2

Se mostrará:
```
[INFO] Saved 33 waypoints to data/waypoints.yaml
```

## ▶️ Paso a Paso: Seguimiento

### Terminal 2: Lanzar Seguimiento (TNT presionar Ctrl+C primero)
```bash
cd ~/ros2_ws && source install/setup.bash
ros2 launch autonomous_patrol follow_waypoints.launch.py
```

**Observa en Gazebo**: El robot debería seguir la ruta que grabaste

**En los logs**:
```
[INFO] Loaded 33 waypoints
[INFO] Starting autonomous waypoint following
[INFO] Waypoint 1 reached! Error: 0.1947m Time: 2.61s Total: 1/33
[INFO] Waypoint 509 reached! Error: 0.1924m Time: 0.70s Total: 2/33
...
[INFO] All waypoints completed!
```

## 📊 Ver Resultados

Después de completar el seguimiento:

```bash
cat ~/ros2_ws/install/autonomous_patrol/share/autonomous_patrol/results/metrics.json | jq
```

Mostrará:
- Tiempo total
- Número de waypoints completados
- Error promedio
- Velocidades utilizadas

## 🔧 Troubleshooting

**P: Los waypoints solo forman una línea recta**
R: Porque grabaste solo en línea recta. Repite el proceso moviendo el robot en formas más complejas.

**P: El robot no se mueve en el following**
R: Verifica que Gazebo siga en marcha. Si reiniciaste Gazebo, primero debe haber waypoints guardados.

**P: El archivo no se guarda**
R: Asegúrate de presionar `Ctrl+C` en la Terminal 2 (es el SIGNAL para guardar).

**P: Quiero cambiar la distancia mínima**
R: Edita `autonomous_patrol/config/autonomous_patrol_config.yaml`:
```bash
nano ~/ros2_ws/src/nomeer_robot_ros2/src/autonomous_patrol/config/autonomous_patrol_config.yaml
```
Cambia `min_distance_between_waypoints: 0.1` al valor deseado, luego recompila:
```bash
cd ~/ros2_ws && colcon build --packages-select autonomous_patrol
```
