# WiFi Quick Commands (Copy-Paste)

Lanzar desde Raspberry Pi **sin relanzar setup**.

---

## 1️⃣ Crear .env con credenciales

**Reemplaza los valores y pega en terminal:**

```bash
SSID="tu-red-wifi"
PASS="tu-contraseña"
cat > ~/.env << EOF
MOWER_WIFI_SSID="$SSID"
MOWER_WIFI_PASSWORD="$PASS"
MOWER_WIFI_CONNECTION_NAME="openmower-wifi"
EOF
chmod 600 ~/.env
```

---

## 2️⃣ Conectar WiFi Ahora

**Opción A: Script directo**
```bash
bash ~/open_mower_ros/utils/scripts/connect_wifi_from_env.sh
```

**Opción B: Servicio systemd**
```bash
sudo systemctl restart openmower-wifi.service
```

---

## 3️⃣ Verificar

```bash
# Estado
sudo systemctl status openmower-wifi.service

# Logs en vivo
sudo journalctl -u openmower-wifi.service -f

# IP actual
hostname -I
```

---

## Una sola línea (credenciales pre-rellenadas)

Edita `SSID` y `PASS`, luego pega:

```bash
SSID="mi-red"; PASS="mi-pass"; cat > ~/.env << EOF
MOWER_WIFI_SSID="$SSID"
MOWER_WIFI_PASSWORD="$PASS"
MOWER_WIFI_CONNECTION_NAME="openmower-wifi"
EOF
chmod 600 ~/.env && bash ~/open_mower_ros/utils/scripts/connect_wifi_from_env.sh
```

---

## Editar después

Cualquier momento:

```bash
nano ~/.env                    # Editar credenciales
bash ~/open_mower_ros/utils/scripts/connect_wifi_from_env.sh  # Reconectar
```

O ver logs:

```bash
tail -f /var/log/openmower-wifi.log
```
