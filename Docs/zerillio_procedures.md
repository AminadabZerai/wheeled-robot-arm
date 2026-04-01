# Zerillio - Raspberry Pi 3 B+ Quick Reference

## Connection Info

- **Hostname:** zerillio.local
- **IP (hotspot):** check with `arp -a` from your PC (changes each time)
- **Username:** pi3
- **SSH command:** `ssh pi3@zerillio.local` or `ssh pi3@<IP_ADDRESS>`

---

## Starting Up

1. Make sure your phone hotspot is ON and broadcasting
2. Plug in the power cable to the Pi (there is no power button)
3. Wait ~2 minutes for boot + Wi-Fi connection
4. Check your phone — it should show a new connected device
5. From your PC (on the same hotspot), open Tabby or PowerShell:
   ```
   ssh pi3@zerillio.local
   ```
6. If hostname doesn't resolve:
   ```
   arp -a
   ```
   Find the Pi's IP and connect with `ssh pi3@<IP>`

---

## Shutting Down (Graceful)

Always shut down gracefully to avoid SD card corruption:

```
sudo shutdown -h now
```

Wait about 10 seconds until the green LED stops blinking, then unplug the power cable.

### Other Shutdown Options

| Command | What it does |
|---|---|
| `sudo shutdown -h now` | Shutdown immediately |
| `sudo shutdown -h +5` | Shutdown in 5 minutes |
| `sudo poweroff` | Same as shutdown -h now |
| `sudo reboot` | Restart the Pi |

---

## NEVER DO THIS

- **Never just unplug the power cable** while the Pi is running — this can corrupt the SD card and break your OS/ROS installation

---

## Useful Commands Once Connected

| Command | What it does |
|---|---|
| `htop` | Monitor CPU/RAM usage |
| `df -h` | Check disk space |
| `vcgencmd measure_temp` | Check CPU temperature |
| `tmux` | Start a persistent terminal session |
| `tmux attach` | Reattach to a tmux session after disconnect |
| `ros2 doctor` | Check ROS 2 health |
| `source ~/.bashrc` | Reload shell config |

---

## Wi-Fi Networks Configured

1. **ABI24** (phone hotspot) — primary, SSH works
2. **eduroam** (Polito) — internet only, SSH blocked by network

---

## ROS 2 Info

- **Distro:** Jazzy Jalisco
- **Workspace:** ~/ros2_ws/src
- **Domain ID:** 42
- **Robot name:** zerillio
