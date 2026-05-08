# WiFi Auto-Connect Guide

Conecta Raspberry automáticamente en boot (sin relanzar setup).

Scripts:
- `connect_wifi_from_env.sh` — Conecta WiFi desde `.env`
- `install_wifi_service.sh` — Instala servicio systemd (sin relanzar setup)
- `setup_raspberry_ubuntu.sh` — Setup completo (instala todo)

---

## Opción A: Setup Completo (primera vez)

```bash
cd ~/open_mower_ros
bash ./scripts/setup_raspberry_ubuntu.sh
```

Instala automáticamente:
- `.env` con plantilla
- Servicio systemd `openmower-wifi.service`

---

## Opción B: Solo Instalar Servicio WiFi (sin relanzar setup)

Si ya ejecutaste setup o solo quieres instalar el servicio:

```bash
bash ~/open_mower_ros/scripts/install_wifi_service.sh
```

O con sudo si es necesario:

```bash
sudo bash ~/open_mower_ros/scripts/install_wifi_service.sh
```

---

## Usar (sin relanzar nada)

### 1. Editar credenciales

```bash
nano ~/open_mower_ros/.env
```

Verifica que tenga:
```env
MOWER_WIFI_SSID="tu-red-wifi"
MOWER_WIFI_PASSWORD="tu-contraseña"
MOWER_WIFI_CONNECTION_NAME="openmower-wifi"
```

### 2. Conectar ahora

**Opción A: Script directo**
```bash
bash ~/open_mower_ros/scripts/connect_wifi_from_env.sh
```

**Opción B: Servicio systemd**
```bash
sudo systemctl restart openmower-wifi.service
```

### 3. Verificar

```bash
# Estado
sudo systemctl status openmower-wifi.service

# Logs en vivo
sudo journalctl -u openmower-wifi.service -f

# IP actual
hostname -I

# Test internet
ping 8.8.8.8
```

---

## Verificar Servicio Instalado

```bash
# Debe mostrar "enabled"
sudo systemctl is-enabled openmower-wifi.service

# Si no está instalado:
bash ~/open_mower_ros/scripts/install_wifi_service.sh
```

---

## Troubleshooting

| Problema | Solución |
|----------|----------|
| `.env` not found | Crea en `~/open_mower_ros/.env` (no en `~/`) |
| Service not found | `bash install_wifi_service.sh` |
| No conecta | Check SSID/password, ver logs: `journalctl -u openmower-wifi.service -n 50` |
| Redes visibles | `nmcli dev wifi list` |

---

## Comandos Útiles

```bash
# Listar redes disponibles
nmcli dev wifi list

# Ver conexiones guardadas
nmcli connection show

# Eliminar conexión guardada
sudo nmcli connection delete openmower-wifi

# Resetear servicio
sudo systemctl restart openmower-wifi.service

# Ver logs
tail -f /var/log/openmower-wifi.log

# Ver IP
hostname -I
```

---

## Boot Automático

Una vez que servicio está `enabled`:

1. **Reboot**: Systemd ejecuta `openmower-wifi.service`
2. **Script corre**: Lee `.env`, configura conexión WiFi
3. **NetworkManager**: Conecta automáticamente
4. **IP recibida**: Listo para usar

✓ Persistente. Sin intervención manual.
