# WiFi Auto-Connect: Manual Usage

Script: `connect_wifi_from_env.sh` y servicio: `openmower-wifi.service`

Sin necesidad de relanzar `setup_raspberry_ubuntu.sh`.

---

## Setup Inicial (primera vez)

```bash
cd ~/open_mower_ros
bash ./utils/scripts/setup_raspberry_ubuntu.sh
```

Crea:
- `~/.env` con plantilla (edita con tus credenciales)
- Servicio systemd `openmower-wifi.service`

---

## Uso Manual (desde terminal)

### 1. Editar credenciales

```bash
nano ~/.env
```

Asegúrate de tener:
```env
MOWER_WIFI_SSID="tu-red-wifi"
MOWER_WIFI_PASSWORD="tu-contraseña"
MOWER_WIFI_CONNECTION_NAME="openmower-wifi"
```

### 2. Conectar ahora (sin esperar reboot)

**Opción A: Ejecutar script directamente**
```bash
bash ~/open_mower_ros/utils/scripts/connect_wifi_from_env.sh
```

**Opción B: Usar servicio systemd**
```bash
sudo systemctl restart openmower-wifi.service
```

### 3. Verificar conexión

```bash
# Estado del servicio
sudo systemctl status openmower-wifi.service

# Ver logs
sudo journalctl -u openmower-wifi.service -f

# Verificar WiFi actual
nmcli connection show --active
ip addr show
```

---

## Troubleshooting

| Problema | Solución |
|----------|----------|
| "No .env file found" | `cat > ~/.env` con credenciales (ver arriba) |
| Servicio falla | Ver logs: `journalctl -u openmower-wifi.service -n 50` |
| No conecta | Check SSID/password en `~/.env`, verifca red visible: `nmcli dev wifi list` |
| Permission denied | Ejecuta con `sudo bash connect_wifi_from_env.sh` o usa servicio |

---

## Variables en .env

| Variable | Descripción |
|----------|------------|
| `MOWER_WIFI_SSID` | Nombre red WiFi |
| `MOWER_WIFI_PASSWORD` | Contraseña WiFi |
| `MOWER_WIFI_CONNECTION_NAME` | Nombre conexión (default: `openmower-wifi`) |

---

## Comandos Útiles

```bash
# Listar redes disponibles
nmcli dev wifi list

# Ver conexiones guardadas
nmcli connection show

# Eliminar conexión
sudo nmcli connection delete openmower-wifi

# Resetear servicio WiFi
sudo systemctl stop openmower-wifi.service
sudo systemctl start openmower-wifi.service

# Ver IP asignada
hostname -I

# Test internet
ping 8.8.8.8
```

---

## Archivo de logs

Cada ejecución se loguea en:
```bash
/var/log/openmower-wifi.log
tail -f /var/log/openmower-wifi.log
```
