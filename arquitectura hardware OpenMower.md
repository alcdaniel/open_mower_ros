# Arquitectura Hardware OpenMower (v1 como referencia para Pi 4 + Arduino Mega)

Fecha de análisis: 2026-05-07  
Workspace analizado: `ros/open_mower_ros`

## 1) Alcance y contexto
Este documento resume la arquitectura hardware/comunicaciones de OpenMower v1 para usarla como referencia en una migración de un robot tipo Repel/ReP_AL Mower Monster 220 hacia:

- Raspberry Pi 4 + ROS (alto nivel)
- GPS RTK (ArduSimple/u-blox ZED-F9P)
- Arduino Mega (bajo nivel, seguridad y actuadores)

Además, se incluye una propuesta técnica concreta para una primera versión funcional y segura.

## 2) Qué está verificado en este repositorio (y qué no)

### 2.1 Verificado en código de este repo
- `open_mower.launch` incluye `_comms.launch` para la capa de comunicaciones: `src/open_mower/launch/open_mower.launch:2`.
- En este fork, `_comms.launch` selecciona por `HARDWARE_PLATFORM`:
  - `1` -> `mower_comms_v1` (`src/open_mower/launch/include/_comms.launch:8-12`)
  - `2` -> `mower_mega_bridge` (`src/open_mower/launch/include/_comms.launch:29-38`)
- El nodo v1 (`mower_comms_v1`) usa serie con `serial::Serial`, COBS, CRC-CCITT, delimitador `0x00`, y 115200 bps (`src/mower_comms_v1/src/mower_comms.cpp:25`, `33-35`, `790`, `819-836`).
- Los tipos de paquete binario y estructuras están en `ll_datatypes.h` (`src/mower_comms_v1/src/ll_datatypes.h:21-27`, `54-198`).
- En v1, `/ll/cmd_vel` se convierte a velocidades izquierda/derecha y se aplica a `xesc_driver` (no como paquete de velocidad al low-level): `src/mower_comms_v1/src/mower_comms.cpp:471-487`, `136-143`.

### 2.2 No verificado en este repo (contexto externo)
- Detalles físicos de la placa xCore (CM4, STM32H723, switch Ethernet, etc.) no aparecen explícitos en este árbol local.
- Se mantienen aquí como contexto externo aportado por ti.

## 3) Arquitectura OpenMower v1 (alto/bajo nivel)

### 3.1 Separación funcional
- Alto nivel (Raspberry Pi 4 + ROS): navegación, planificación, lógica de estado, GPS, servicios/app.
- Bajo nivel (low-level board/Pico): señales críticas, seguridad local, sensores de emergencia/UI/IMU/power.
- El puente ROS <-> low-level en v1 es `mower_comms_v1`.

### 3.2 Esquema general (v1)
```text
[Raspberry Pi 4 | Ubuntu 20.04 | ROS Noetic | open_mower_ros]
         |
         | ROS topics/services
         |  - /ll/cmd_vel
         |  - /ll/_service/emergency
         |  - /ll/_service/mow_enabled
         |  - /mower_logic/current_state
         v
[mower_comms_v1]
         |
         | serial::Serial @115200
         | COBS + CRC-CCITT + 0x00 (fin de trama)
         v
[Low-Level Board / Pico]
         |
         | emergencia, power, rain, UI, IMU, estado
         v
[Topics ROS: /ll/mower_status, /ll/power, /ll/emergency, /ll/imu/data_raw]
```

### 3.3 Rama paralela de tracción/cuchilla en v1
```text
/ll/cmd_vel --> mower_comms_v1 --> xesc_driver (left/right/mower)
                                   |         |         |
                                   v         v         v
                                xESC L    xESC R    xESC Mow
                                   \        |        /
                                   motores ruedas + cuchilla
```

## 4) Selección de plataforma y diferencia v1 vs v2 (en este código)

### 4.1 Selección por launch
- `_comms.launch`:
  - `HARDWARE_PLATFORM==1`: `mower_comms_v1`.
  - `HARDWARE_PLATFORM==2`: `mower_mega_bridge` en este fork.
  - Fuente: `src/open_mower/launch/include/_comms.launch:8-12`, `29-38`.

### 4.2 Señal de arquitectura v2
- Existe paquete `mower_comms_v2` que usa interfaces de servicio (`*ServiceInterface`) y `target_add_service_interface(...)` en CMake:
  - `src/mower_comms_v2/CMakeLists.txt:185-192`
  - `src/mower_comms_v2/src/mower_comms.cpp:31-38`, `151-152`, `192-270`
- Esto confirma que v2 no usa el mismo flujo binario serie COBS/CRC de v1.

## 5) Comunicación Raspberry Pi <-> low-level en v1

## 5.1 Capa física/lógica
- `ll_serial_port` llega por parámetro ROS (`mower_comms_v1`): `src/mower_comms_v1/src/mower_comms.cpp:725-729`.
- 115200 bps fijo en código: `src/mower_comms_v1/src/mower_comms.cpp:790`.
- Para software, puede ser `/dev/ttyACM*`, `/dev/ttyUSB*`, `/dev/ttyAMA*`, `/dev/serial/by-id/...`.
- En perfiles hardware específicos se ve, por ejemplo, `ll_serial_port: /dev/ttyAMA0`: `src/open_mower/params/hardware_specific/YardForce500/comms_general_params.yaml:1`.

## 5.2 Framing, checksum y parseo
- COBS encode/decode: `src/mower_comms_v1/src/COBS.h:35-62`, `70-97`.
- Delimitador fin de paquete: byte `0x00` añadido tras COBS: `src/mower_comms_v1/src/mower_comms.cpp:154-155`, `362-363`, `459-460`.
- Recepción byte a byte hasta `0x00`, luego decode COBS: `src/mower_comms_v1/src/mower_comms.cpp:807`, `819-823`.
- Integridad con CRC-CCITT (`boost::crc_ccitt_type`): `src/mower_comms_v1/src/mower_comms.cpp:91`, `830-836`.

### 5.3 Formato de trama (v1)
Antes de COBS:
```text
[type][payload...][crc_lo][crc_hi]
```
Después de COBS:
```text
[cobs_encoded_bytes...][0x00]
```
Recepción:
1. Leer hasta `0x00`.
2. `COBS::decode`.
3. Verificar CRC.
4. `buffer_decoded[0]` = `packet_id`.
5. `switch(packet_id)` al handler correspondiente.

(Ver `src/mower_comms_v1/src/mower_comms.cpp:819-863`)

## 6) Tabla de paquetes v1

### 6.1 IDs y estructuras
Definidos en `src/mower_comms_v1/src/ll_datatypes.h:21-27`, `54-198`.

| ID | Dirección | Estructura/uso |
|---|---|---|
| `PACKET_ID_LL_STATUS` (1) | Low-level -> Pi | `ll_status` |
| `PACKET_ID_LL_IMU` (2) | Low-level -> Pi | `ll_imu` |
| `PACKET_ID_LL_UI_EVENT` (3) | Low-level -> Pi | `ll_ui_event` |
| `PACKET_ID_LL_HIGH_LEVEL_CONFIG_REQ` (0x11) | Bidireccional | `ll_high_level_config` + request |
| `PACKET_ID_LL_HIGH_LEVEL_CONFIG_RSP` (0x12) | Bidireccional | `ll_high_level_config` response |
| `PACKET_ID_LL_HEARTBEAT` (0x42) | Pi -> Low-level | `ll_heartbeat` |
| `PACKET_ID_LL_HIGH_LEVEL_STATE` (0x43) | Pi -> Low-level | `ll_high_level_state` |

### 6.2 Pi -> Low-level (v1)

#### `LL_HEARTBEAT` (0x42)
- Campos: `emergency_requested`, `emergency_release_requested`.
- Envío cíclico desde `publishActuators()`: `src/mower_comms_v1/src/mower_comms.cpp:144-160`.

#### `LL_HIGH_LEVEL_STATE` (0x43)
- Campos: `current_mode`, `gps_quality`.
- Se forma desde `/mower_logic/current_state`: `src/mower_comms_v1/src/mower_comms.cpp:449-453`, `458-465`.

#### `LL_HIGH_LEVEL_CONFIG_REQ` (0x11)
- Contenido: `ll_high_level_config` (batería, carga, lluvia, hall/lift/tilt, sonido/UI, etc.).
- Envío y reintentos: `src/mower_comms_v1/src/mower_comms.cpp:338-417`.

### 6.3 Low-level -> Pi (v1)

#### `LL_STATUS` (1)
- Handler: `handleLowLevelStatus(...)`: `src/mower_comms_v1/src/mower_comms.cpp:607-617`.
- Datos relevantes (struct):
  - `status_bitmask`: init, power, charging, rain, sound, UI (`ll_datatypes.h:43-50`, `66`).
  - `uss_ranges_m[5]`, `emergency_bitmask`, `v_charge`, `v_system`, `charging_current`, `batt_percentage` (`ll_datatypes.h:68-81`).
- Publicación ROS en `publishStatus()`: `src/mower_comms_v1/src/mower_comms.cpp:206-253`.

#### `LL_IMU` (2)
- Handler: `handleLowLevelIMU(...)`: `src/mower_comms_v1/src/mower_comms.cpp:619-643`.
- Campos: accel, gyro, mag, dt (`ll_datatypes.h:86-96`).

#### `LL_UI_EVENT` (3)
- Handler: `handleLowLevelUIEvent(...)`: `src/mower_comms_v1/src/mower_comms.cpp:489-530`.
- Mapea botones Home/Play/S1/S2/Lock a `mower_service/high_level_control`.

#### `LL_HIGH_LEVEL_CONFIG_RSP` (0x12)
- Handler: `handleLowLevelConfig(...)`: `src/mower_comms_v1/src/mower_comms.cpp:558-605`.
- Sincroniza config LL <-> `mower_logic`/`power` vía dynamic_reconfigure.

## 7) Qué viaja por serie v1 y qué NO viaja por serie

### 7.1 Sí viaja por serie Pi<->LL
- Heartbeat y latch/release de emergencia.
- Estado high-level (modo y calidad GPS).
- Configuración LL/HL (power/rain/hall/sound...).
- Telemetría LL (status/power/rain/emergency/UI/IMU).

### 7.2 No viaja (en v1) como “cmd_vel directo” hacia Pico
- `/ll/cmd_vel` se convierte a `speed_l/speed_r` en Pi:
  - `speed_r = linear.x + 0.5*wheel_distance_m*angular.z`
  - `speed_l = linear.x - 0.5*wheel_distance_m*angular.z`
  - Fuente: `src/mower_comms_v1/src/mower_comms.cpp:474-475`.
- Esos mandos se aplican por `xesc_driver::setDutyCycle(...)`:
  - Izq, dcha (invertida), y cuchilla: `src/mower_comms_v1/src/mower_comms.cpp:136-143`.

Conclusión: en v1, la low-level board no es el canal principal de control de tracción xESC; actúa más como supervisor de seguridad/sensores/estado.

## 8) Ciclo temporal y robustez
- Timer principal cada 0.02 s (50 Hz): `src/mower_comms_v1/src/mower_comms.cpp:774`.
- En cada ciclo:
  - `publishActuators()`
  - `publishStatus()`
  - `configTracker.check()`
  - Fuente: `src/mower_comms_v1/src/mower_comms.cpp:420-424`.
- Watchdogs/paradas:
  - Sin `cmd_vel` >1 s: ruedas a 0 (`126-129`).
  - Sin `cmd_vel` >25 s: ruedas y cuchilla a 0 (`130-134`).
- Se asume status LL cada ~100 ms; si se “pierde” ~1 s, se marca config dirty (reinicio/flash LL): `src/mower_comms_v1/src/mower_comms.cpp:613-616`.

## 9) Esquema de comunicaciones y conexiones físicas (Pi 4 <-> Pico)

## 9.1 Conexiones físicas posibles
```text
Opción A (muy práctica): USB CDC
Raspberry Pi 4 USB  <---- cable USB ---->  Pico USB
Dispositivo Linux típico: /dev/ttyACM0

Opción B (UART TTL)
Pi TX/RX/GND  <---- UART TTL ---->  Pico RX/TX/GND
Dispositivo Linux típico: /dev/ttyAMA0, /dev/serial/by-id/...
```

## 9.2 Stack de comunicación (v1)
```text
Aplicación:        paquetes LL_* (status, imu, heartbeat, config, state)
Integridad:        CRC-CCITT (16-bit)
Framing:           COBS
Delimitación:      0x00 al final de trama
Transporte:        serial::Serial
Físico/enlace:     USB CDC o UART TTL
Velocidad típica:  115200 bps (fijo en mower_comms_v1)
```

## 10) GPS RTK dentro de esta arquitectura

### 10.1 Puntos verificados en configuración
- `OM_USE_NTRIP` habilita `_ntrip_client.launch` desde `_comms.launch`: `src/open_mower/launch/include/_comms.launch:43`.
- `OM_GPS_BAUDRATE`, `OM_GPS_PORT`, `OM_GPS_PROTOCOL`, `OM_USE_RELATIVE_POSITION`, `OM_DATUM_LAT/LONG` se mapean en `_params.launch`:
  - `src/open_mower/launch/include/_params.launch:65-80`.
- `OM_GPS_PROTOCOL=UBX` y `OM_USE_RELATIVE_POSITION=False` están en el ejemplo de config:
  - `config/mower_config.sh.example:107`, `110`.
- Ejemplo de override a 921600 y `/dev/serial/by-id/...`: `config/mower_config.sh.example:112-117`.

### 10.2 Lectura práctica para tu caso
- Aunque uses NTRIP externo con RTK FIX, sigue siendo útil mantener `OM_DATUM_LAT/LONG` como origen local del mapa (modo absoluto).
- El DATUM es el origen del mapa local; no es la posición de la estación base RTCM.

## 11) Propuesta adaptada a tu robot (Pi 4 + Mega + RTK)

## 11.1 Arquitectura objetivo (fase 1)
```text
[Raspberry Pi 4 | ROS]
  - navegación/localización/estado/UI
  - nodo bridge ROS<->Mega
  - GPS RTK ZED-F9P (UBX, ideal 921600)
         |
         | USB serial (recomendado inicio)
         | 115200 (arranque) o 921600 (cuando estabilice)
         v
[Arduino Mega]
  - seguridad inmediata (E-STOP local)
  - watchdog heartbeat/cmd timeout
  - control motores (PWM/driver)
  - cuchilla
  - bumpers, sonars, perímetro, rain, tilt
         v
[Actuadores y sensores]
```

## 11.2 Principio clave de seguridad
- Si no llega heartbeat/cmd en X ms -> Mega fuerza motores a cero.
- Si hay emergencia física -> Mega para motores aunque la Pi siga viva.
- La Pi decide misión y navegación; el Mega manda en seguridad instantánea.

## 11.3 Protocolo mínimo recomendado (inspirado en v1, adaptado a Mega)

### Pi -> Mega
- `HEARTBEAT`
- `CMD_VEL(linear, angular)` o `CMD_MOTOR(left,right)`
- `SET_MOWER_ENABLE(bool)`
- `CLEAR_EMERGENCY`
- `REQUEST_STATUS`

### Mega -> Pi
- `STATUS`
- `BATTERY`
- `EMERGENCY`
- `SENSOR_STATE`
- `PERIMETER_SIGNAL`
- `WHEEL_TICKS` (cuando agregues encoders/hall)
- `IMU` (si reside en Mega)

### Recomendación de framing
- Opción simple inicial (rápida de depurar):
  - Texto tipo NMEA: `$TYPE,f1,f2,...*CS\n` (XOR checksum)
- Opción robusta final (estilo OpenMower v1):
  - Binario + COBS + CRC16 + `0x00`

## 11.4 Timeouts sugeridos (fase 1)
- `cmd_vel timeout`: 300-500 ms -> parar ruedas.
- `heartbeat timeout`: 1000 ms -> entrar en estado seguro.
- `emergency latch`: sólo se limpia con comando explícito + condición segura.

## 12) Plan de implementación recomendado (primera versión funcional y segura)

1. Crear/usar un bridge ROS<->Mega con interfaz ROS equivalente a `ll/*`.
2. Implementar en Mega:
   - parser robusto,
   - watchdog,
   - E-STOP latched,
   - telemetría base (batería/estado/bumper/sonar/perímetro).
3. Integrar GPS RTK (ZED-F9P) en Pi con UBX y NTRIP.
4. Probar en banco:
   - pérdida de enlace serie,
   - bloqueo de ROS,
   - emergencia física,
   - reinicio de Pi/Mega.
5. Subir baudrate/protocolo cuando la base sea estable.

## 13) Observación relevante para tu fork actual
En este repositorio ya existe un puente Pi<->Mega (`mower_mega_bridge`) que publica gran parte de la interfaz `ll/*` y usa protocolo serial textual con checksum XOR (`src/mower_mega_bridge/src/mower_mega_bridge_node.cpp:4-6`, `81-86`, `368-383`, `1145-1150`).

Esto puede acelerar mucho tu fase 1 porque ya sigue el mismo patrón arquitectónico que buscas (alto nivel ROS en Pi + bajo nivel en Mega).

---

## Referencias rápidas (archivos clave)
- `src/open_mower/launch/open_mower.launch`
- `src/open_mower/launch/include/_comms.launch`
- `src/open_mower/launch/include/_params.launch`
- `src/mower_comms_v1/src/mower_comms.cpp`
- `src/mower_comms_v1/src/ll_datatypes.h`
- `src/mower_comms_v1/src/COBS.h`
- `src/mower_comms_v2/src/mower_comms.cpp`
- `src/mower_comms_v2/CMakeLists.txt`
- `src/open_mower/params/hardware_specific/YardForce500/comms_general_params.yaml`
- `config/mower_config.sh.example`
- `src/mower_mega_bridge/src/mower_mega_bridge_node.cpp`

---

# Anexo GPS: OpenMower + ArduSimple/u-blox ZED-F9P (solo OpenMower)

Este anexo se centra únicamente en cómo OpenMower integra el GPS RTK (rover), sin hablar de adaptaciones externas.

## A1) Arquitectura general GPS en OpenMower

```text
Raspberry Pi 4
Ubuntu 20.04 + ROS Noetic + OpenMower
        |
        | USB o UART serial
        | GPS -> Raspberry: UBX/NMEA
        | Raspberry -> GPS: RTCM3 (si usa NTRIP)
        v
ArduSimple / u-blox ZED-F9P (rover RTK)
        |
        | antena GNSS
        v
satélites GNSS
```

Si se usa NTRIP:

```text
NTRIP caster / servidor RTCM
        |
        | RTCM3 por internet
        v
Raspberry Pi 4 / OpenMower (cliente NTRIP)
        |
        | RTCM3 por serial (USB/UART)
        v
ZED-F9P (rover)
        |
        | cálculo interno de solución RTK
        v
OpenMower recibe posición precisa
```

Puntos clave:
- El ZED-F9P calcula internamente la solución GNSS/RTK.
- OpenMower no resuelve RTK “a mano”; consume la posición/estado que entrega el receptor.
- La Raspberry puede actuar como cliente NTRIP (`_ntrip_client.launch`).
- Las correcciones RTCM deben llegar físicamente al ZED-F9P por el enlace serial.
- Sin RTCM al receptor, habrá GNSS normal (3D/DGPS), pero no RTK Fixed.

## A2) Conexión física GPS <-> Raspberry Pi 4

### A2.1 USB directo
- Enlace: USB Pi 4 <-> USB ArduSimple.
- Dispositivos Linux típicos: `/dev/ttyACM0`, `/dev/ttyUSB0`, `/dev/serial/by-id/...`.
- Recomendación práctica en OpenMower: usar `/dev/serial/by-id/...` para estabilidad de nombre.
- Ventajas:
  - cableado simple,
  - bidireccional (UBX/NMEA de salida + RTCM3 de entrada),
  - sin configuración adicional de UART GPIO.
- Inconveniente típico: asegurar mecánicamente el conector por vibración.

### A2.2 UART
- Enlace: GPIO UART Pi <-> UART1/UART2 GPS.
- Señales:
  - TX Pi -> RX GPS,
  - RX Pi <- TX GPS,
  - GND común.
- Requiere:
  - habilitar UART en Raspberry,
  - desactivar consola serie,
  - fijar baudrate,
  - comprobar niveles lógicos.
- Dispositivos Linux típicos: `/dev/ttyAMA0`, `/dev/ttyS0`.
- En OpenMower normalmente USB es la opción más simple para arrancar.

## A3) Protocolos implicados

### A3.1 UBX
- Binario propietario u-blox.
- En OpenMower, con F9P, se usa como protocolo recomendado (`OM_GPS_PROTOCOL=UBX`).
- Transporta estado de fix, posición, velocidad y calidad con menor ambigüedad que NMEA.

### A3.2 NMEA
- ASCII estándar (`GGA`, `RMC`, etc.).
- Útil para diagnóstico.
- No imprescindible para operación principal si OpenMower usa UBX, pero práctico para debug.

### A3.3 RTCM3
- Correcciones diferenciales RTK.
- Deben entrar al F9P.
- En OpenMower, suelen llegar por NTRIP a la Raspberry y reenviarse al puerto GPS.

### A3.4 NTRIP
- Transporte por red de RTCM.
- OpenMower (Raspberry) se conecta al caster y recibe RTCM3.
- NTRIP no sustituye UBX/NMEA local; es el canal de correcciones.

## A4) Configuración recomendada en u-center (ZED-F9P para OpenMower)

## A4.1 RATE
Ruta: `View -> Configuration View -> RATE`

- `Measurement Period = 200 ms`
- `Navigation Rate = 1`
- `Time Reference = GPS`

Resultado: 5 Hz de navegación.

Notas:
- 1 Hz suele ser bajo para robot móvil.
- 5 Hz es un compromiso habitual.
- 10 Hz puede funcionar, con más tráfico/carga.

## A4.2 PRT
Ruta: `View -> Configuration View -> PRT`

Para USB:
- `Target = USB`
- `Protocol in = UBX + NMEA + RTCM3`
- `Protocol out = UBX + NMEA`

Motivo:
- `UBX out`: OpenMower lee navegación/estado.
- `RTCM3 in`: receptor recibe correcciones desde Raspberry.
- `NMEA out`: útil para diagnóstico.

Para UART (si se usa):
- `Target = UART1` o `UART2`
- `Baudrate recomendado = 921600` (si se necesita margen alto)
- `Protocol in/out` equivalente al flujo requerido.

## A4.3 MSG
Ruta: `View -> Configuration View -> MSG`

En el puerto hacia Raspberry (normalmente USB):
- Activar `UBX-NAV-PVT (Class 0x01, ID 0x07)` con tasa 1 por ciclo de navegación.

Opcionales de diagnóstico:
- `NAV-SAT`
- `NAV-STATUS`
- `NAV-DOP`
- `NMEA-GGA`
- `NMEA-RMC`

## A4.4 Guardar configuración
Ruta: `View -> Configuration View -> CFG`

- `Save current configuration`
- Destinos: `BBR + Flash`
- `Send`

Si no se guarda, RATE/PRT/MSG pueden perderse tras reinicio.

## A5) Variables OpenMower relacionadas con GPS

Fuente principal: `config/mower_config.sh.example`.

- `OM_USE_NTRIP` (`config/mower_config.sh.example:34`)
- `OM_NTRIP_HOSTNAME` (`:35`)
- `OM_NTRIP_PORT` (`:36`)
- `OM_NTRIP_USER` (`:37`)
- `OM_NTRIP_PASSWORD` (`:38`)
- `OM_NTRIP_ENDPOINT` (`:39`)
- `OM_USE_RELATIVE_POSITION` (`:107`)
- `OM_DATUM_LAT` (`:26`)
- `OM_DATUM_LONG` (`:27`)
- `OM_GPS_PROTOCOL` (`:110`)
- `OM_GPS_BAUDRATE` ejemplo (`:116`)
- `OM_GPS_PORT` ejemplo (`:117`)
- `OM_USE_F9R_SENSOR_FUSION` (`:122`)

Además, `_params.launch` inyecta estas variables al parameter server ROS:
- `ll/services/gps/baudrate` desde `OM_GPS_BAUDRATE` (`src/open_mower/launch/include/_params.launch:65-66`)
- `ll/services/gps/serial_port` desde `OM_GPS_PORT` (`:67`)
- `ll/services/gps/mode` según `OM_USE_RELATIVE_POSITION` (`:73-74`)
- `ll/services/gps/protocol` desde `OM_GPS_PROTOCOL` (`:75`)
- `ll/services/gps/datum_lat/long/height` (`:76-80`)

Carga base de parámetros GPS en v1:
- `_params.launch` carga `comms_gps_params.yaml` en `ll/services/gps` (`src/open_mower/launch/include/_params.launch:22-25`).
- Ejemplo de defaults por hardware: `serial_port: "/dev/ttyAMA2"` y `baudrate: 921600` en `src/open_mower/params/hardware_specific/YardForce500/comms_gps_params.yaml:1-2`.

Interpretación operativa:
- Con `OM_USE_RELATIVE_POSITION=False`, `OM_DATUM_LAT/LONG` siguen siendo obligatorios como origen local del mapa.
- El DATUM no es “la base NTRIP”; es referencia local de coordenadas para OpenMower.

## A6) Flujo de nodos/topics GPS en OpenMower

En v1 (`HARDWARE_PLATFORM=1`) `_comms.launch` arranca el driver GPS `xbot_driver_gps`:
- Nodo: `driver_gps_node` (`src/open_mower/launch/include/_comms.launch:14-19`)
- Remaps:
  - `/ll/services/rtcm` -> `/ll/position/gps/rtcm` (`:20`)
  - `/nmea` -> `/ll/position/gps/nmea` (`:21`)
  - `~xb_pose` -> `/ll/position/gps` (`:22`)
  - `~wheel_ticks` -> `/mower/wheel_ticks` (`:23`)

`open_mower.launch` incluye `_comms.launch` (`src/open_mower/launch/open_mower.launch:2`).

Flujo conceptual:

```text
GPS ZED-F9P
   |
   | UBX/NMEA
   v
xbot_driver_gps (driver_gps_node)
   |
   +--> /ll/position/gps
   +--> /ll/position/gps/nmea

Si NTRIP activo:
ntrip_watchdog / ntrip client
   |
   | RTCM
   v
/ll/position/gps/rtcm
   |
   v
xbot_driver_gps escribe RTCM al puerto serie del GPS
   |
   v
ZED-F9P mejora solución (Float/Fixed)
```

El include de NTRIP se activa en `_comms.launch` con `OM_USE_NTRIP` (`src/open_mower/launch/include/_comms.launch:43`).

## A7) Comandos Linux útiles (Raspberry)

Listar puertos serie:
```bash
ls -l /dev/serial/by-id/
ls -l /dev/ttyACM*
ls -l /dev/ttyUSB*
dmesg | grep -i tty
dmesg | grep -i usb
```

Permisos:
```bash
groups
sudo usermod -a -G dialout $USER
```

Lectura básica:
```bash
cat /dev/ttyACM0
```

Si necesitas fijar baudrate explícito (sobre todo UART):
```bash
stty -F /dev/ttyACM0 921600
cat /dev/ttyACM0
```

Buscar GGA cuando NMEA está habilitado:
```bash
grep --line-buffered GGA /dev/ttyACM0
```

Nota:
- Si sale “basura” en `cat`, puede ser normal cuando el flujo principal es UBX binario.

## A8) Tabla resumen de comunicación

| Enlace | Medio físico | Protocolo | Dirección | Función |
|---|---|---|---|---|
| Raspberry <-> GPS | USB/UART serial | UBX/NMEA | GPS -> Raspberry | Posición, velocidad, estado de fix/calidad |
| Raspberry <-> GPS | USB/UART serial | RTCM3 | Raspberry -> GPS | Correcciones RTK |
| Raspberry <-> NTRIP caster | Internet | NTRIP (transporta RTCM3) | caster -> Raspberry | Correcciones diferenciales |
| GPS <-> satélites | RF GNSS | señales GNSS L1/L2 | satélites -> GPS | Observaciones para solución GNSS/RTK |

## A9) Checklist de validación GPS en OpenMower

- [ ] ZED-F9P conectado físicamente a Raspberry (preferible USB al inicio).
- [ ] El dispositivo aparece en `/dev/serial/by-id/...`.
- [ ] `OM_GPS_PORT` apunta a ese puerto estable.
- [ ] `OM_GPS_PROTOCOL=UBX`.
- [ ] `OM_GPS_BAUDRATE=921600` (o coherente con tu puerto configurado).
- [ ] u-center: `RATE` en 200 ms (5 Hz).
- [ ] u-center: `NAV-PVT` habilitado en el puerto hacia Raspberry.
- [ ] u-center: `Protocol out` incluye UBX.
- [ ] u-center: `Protocol in` incluye RTCM3.
- [ ] u-center: guardado en `BBR + Flash`.
- [ ] NTRIP: host, puerto, usuario, password y mountpoint correctos.
- [ ] Flujo RTCM llega al GPS (`/ll/position/gps/rtcm` activo cuando aplica).
- [ ] El receptor evoluciona a RTK Float/RTK Fixed cuando hay correcciones válidas.
- [ ] OpenMower recibe posición en `/ll/position/gps`.
- [ ] Si usas NMEA-GGA para diagnóstico, refleja estado coherente de fix RTK.

## A10) Qué espera leer OpenMower/ROS desde el GPS

En operación normal, OpenMower espera del driver GPS:
- Pose/posición publicada en `/ll/position/gps` (remap de `~xb_pose`).
- NMEA en `/ll/position/gps/nmea` cuando está habilitado.
- Canal de entrada RTCM por `/ll/position/gps/rtcm` para reenviar correcciones al receptor.

Referencias directas:
- `src/open_mower/launch/include/_comms.launch:14-23`
- `src/open_mower/launch/include/_comms.launch:43`
- `src/open_mower/launch/include/_params.launch:65-80`
- `config/mower_config.sh.example:26-27`, `34-39`, `107`, `110`, `116-117`, `122`
