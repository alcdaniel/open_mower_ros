# WiFi: Quick Commands (Copy-Paste Ready)

Lanzar desde Raspberry Pi sin relanzar setup.

---

## 1️⃣ Crear .env con credenciales

⚠️ **Importante**: Archivo va en `~/open_mower_ros/.env` (NO en `~/`)

```bash
SSID="tu-red-wifi"
PASS="tu-contraseña"
cat > ~/open_mower_ros/.env << EOF
MOWER_WIFI_SSID="$SSID"
MOWER_WIFI_PASSWORD="$PASS"
MOWER_WIFI_CONNECTION_NAME="openmower-wifi"
EOF
chmod 600 ~/open_mower_ros/.env
```

---

## 2️⃣ Instalar Servicio WiFi

Sin relanzar `setup_raspberry_ubuntu.sh`:

```bash
bash ~/open_mower_ros/scripts/install_wifi_service.sh
```

O con sudo si pide:

```bash
sudo bash ~/open_mower_ros/scripts/install_wifi_service.sh
```

---

## 3️⃣ Conectar WiFi Ahora

**Opción A: Script directo**
```bash
bash ~/open_mower_ros/scripts/connect_wifi_from_env.sh
```

**Opción B: Servicio systemd (si instalado)**
```bash
sudo systemctl restart openmower-wifi.service
```

---

## 4️⃣ Verificar Conexión

```bash
# Estado del servicio
sudo systemctl status openmower-wifi.service

# Logs en vivo
sudo journalctl -u openmower-wifi.service -f

# IP asignada
hostname -I

# Test internet
ping 8.8.8.8
```

---

## Todo en Una Línea (completo)

Edita `SSID` y `PASS`, pega:

```bash
SSID="mi-red"; PASS="mi-pass"; cat > ~/open_mower_ros/.env << EOF
MOWER_WIFI_SSID="$SSID"
MOWER_WIFI_PASSWORD="$PASS"
MOWER_WIFI_CONNECTION_NAME="openmower-wifi"
EOF
chmod 600 ~/open_mower_ros/.env && \
bash ~/open_mower_ros/scripts/install_wifi_service.sh && \
bash ~/open_mower_ros/scripts/connect_wifi_from_env.sh
```

---

## Editar Después

Cualquier momento:

```bash
# Editar credenciales
nano ~/open_mower_ros/.env

# Reconectar
bash ~/open_mower_ros/scripts/connect_wifi_from_env.sh

# O restart servicio
sudo systemctl restart openmower-wifi.service

# Ver logs
sudo journalctl -u openmower-wifi.service -f
```

---

## Verificar Servicio

```bash
# Está instalado?
sudo systemctl is-enabled openmower-wifi.service

# Si falta: instalar
bash ~/open_mower_ros/scripts/install_wifi_service.sh
```

---

## Next Reboot

Servicio ejecuta automáticamente → Conecta WiFi → IP recibida ✓
