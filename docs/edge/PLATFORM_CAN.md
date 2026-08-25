# RK3588 / LubanCat SocketCAN bringup for A3 arm

> **Status:** active  
> **产品线：** A3 Edge（`edge`）

This mirrors the verified trotbot (LubanCat-4-V1) setup. Arm traffic normally uses **can0 only**.

## 1. Device tree overlays

Edit `/boot/firmware/ubuntuEnv.txt` (path may vary by image):

```bash
sudo cp /boot/firmware/ubuntuEnv.txt /boot/firmware/ubuntuEnv.txt.bak.$(date +%Y%m%d-%H%M%S)
sudo sed -i 's#^fdtfile=.*#fdtfile=rk3588s-lubancat-4-v1.dtb#' /boot/firmware/ubuntuEnv.txt
sudo sed -i 's#^overlay_prefix=.*#overlay_prefix=#' /boot/firmware/ubuntuEnv.txt
sudo sed -i 's#^overlays=.*#overlays=rk3588s-lubancat-dp0-in-vp1-overlay rk3588s-lubancat-4-hdmi0-overlay rk3588-lubancat-can0-m0-overlay rk3588-lubancat-can2-m0-overlay#' /boot/firmware/ubuntuEnv.txt
sudo reboot
```

Confirm:

```bash
ip -br link | grep can
# expect can0 (and optionally can1 from CAN2 overlay)
```

## 2. Install can-up.service

```bash
sudo cp systemd/can-up.service /etc/systemd/system/can-up.service
sudo systemctl daemon-reload
sudo systemctl enable --now can-up.service
ip -details link show can0
```

Bitrate: **1 Mbps**, `txqueuelen 1000`.

## 3. Single-motor smoke test

```bash
sudo apt-get install -y can-utils
candump -tz can0 &
IFACE=can0 MOTOR_IDS="1" bash src/a3_can_bridge/scripts/a3_motor_cansend.sh check
IFACE=can0 MOTOR_IDS="1" bash src/a3_can_bridge/scripts/a3_motor_cansend.sh enable
IFACE=can0 MOTOR_IDS="1" bash src/a3_can_bridge/scripts/a3_motor_cansend.sh mit
```

Motor IDs for A3: **1..7** (host master `0xFD`).

## 4. Notes

- `can-up.service` cannot create missing interfaces; overlays must succeed first.
- Do not run MotorBridge and `a3_can_bridge` on the same `can0` simultaneously.
- External CAN transceiver required if the carrier board only exposes TX/RX (same as trotbot 40PIN).

## 相关文档

- [ARCHITECTURE.md](ARCHITECTURE.md)
- [QUICKSTART.md](QUICKSTART.md)
- [文档索引](../README.md)
