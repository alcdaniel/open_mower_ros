# Mega Bridge Calibration & Setup

> **Nota:** Todo en C++ para máxima integración tiempo real. Sin Python.

## 1. Calibrar `ODOM_VX_GAIN` (nodo C++)

Sin encoders, la odometría depende de `ODOM_VX_GAIN` (default `0.85`) que estima
qué fracción de la velocidad comandada se ejecuta realmente.

### Procedimiento (con cinta métrica — más preciso)

```bash
# Terminal 1: bridge corriendo
roslaunch mower_mega_bridge bringup.launch

# Terminal 2: calibración C++
rosrun mower_mega_bridge calibrate_odom_gain_node \
    _vx:=0.5 _duration:=8.0 _tape:=4.20 _current_gain:=0.85
```

El nodo:
1. Espera `/odom` (5s timeout)
2. Cuenta atrás 3s
3. Comanda `vx=0.5 m/s` durante `8s` por `/ll/manual_cmd_vel` (mode=1)
4. Lee odom inicial/final
5. Compara con `_tape:=4.20` (medida real con cinta)
6. Imprime nuevo valor de `ODOM_VX_GAIN`

### Ejemplo salida

```
================ CALIBRATION REPORT ================
  current ODOM_VX_GAIN .... 0.850
  odom reported .......... 3.892 m
  real distance (tape)    4.200 m
  ratio (real/reported) .. 1.0791
  ----------------------------------------------
  RECOMMENDED ODOM_VX_GAIN  0.917
====================================================
```

### Aplicar
1. Editar `arduino/MEGA_V9.751/Movement_Control.ino`:
   ```cpp
   static float ODOM_VX_GAIN = 0.917;  // <-- nuevo valor
   ```
2. Reflashear Mega
3. Re-correr calibración para verificar (ratio debe estar cerca de 1.000)

### Con GPS en lugar de cinta
```bash
rosrun mower_mega_bridge calibrate_odom_gain_node \
    _use_gps:=true _gps_topic:=/fix _vx:=0.5 _duration:=15.0 _current_gain:=0.85
```
Recomendado solo con RTK fix; GPS estándar tiene ~3-5 m error → ratio impreciso.

### Parámetros (todos `_param:=value`)

| Param          | Tipo    | Default | Descripción                                    |
|----------------|---------|---------|------------------------------------------------|
| `vx`           | double  | 0.5     | Velocidad lineal (m/s)                         |
| `duration`     | double  | 8.0     | Segundos de comando                            |
| `gps_topic`    | string  | /fix    | Topic GPS (NavSatFix)                          |
| `use_gps`      | bool    | false   | Usar GPS como referencia                       |
| `tape`         | double  | -1.0    | Distancia medida cinta (m); >0 activa modo cinta |
| `current_gain` | double  | 0.85    | Valor actual de `ODOM_VX_GAIN` flasheado      |

### Ctrl-C seguro
El nodo registra `SIGINT` handler que envía `Twist(0,0,0)` 5 veces antes de salir.
Mower se detiene de inmediato si interrumpes.

---

## 2. Localización completa (GPS + IMU + odom)

```bash
roslaunch mower_mega_bridge localization.launch gps_topic:=/fix
```

**Topics resultantes:**
- `/odometry/filtered` → para Nav2 local planner (odom → base_link)
- `/odometry/filtered_map` → para Nav2 global planner (map → odom)
- TF tree: `map → odom → base_link`

---

## 3. Test funcional Nav2 vs Manual

### Manual (con heading correction PID, mode=1)
```bash
rostopic pub -r 10 /ll/manual_cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.3}, angular: {z: 0.0}}'
```

### Nav2 (sin heading PID, pure pursuit, mode=0)
```bash
rostopic pub -r 10 /ll/cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.3}, angular: {z: 0.0}}'
```

### Verificar topics
```bash
rostopic hz /odom              # debe ser ~10 Hz cuando hay MOV activo
rostopic echo /odometry/filtered_map  # GPS-corrected pose
rosrun rqt_console rqt_console  # buscar "[mega_bridge] ACK received"
```

---

## 4. Timeout safety test

```bash
timeout 1 rostopic pub -r 10 /ll/cmd_vel geometry_msgs/Twist '{linear: {x: 0.3}}'
```
Mower debe parar a los ~500ms tras último mensaje (`MOV_COMMAND_TIMEOUT_MS`
en `ROS_Command_Handler.ino`).
