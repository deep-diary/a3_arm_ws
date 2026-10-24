# LL-182 — RK3588 SD 卡系统瘦身克隆成可烧录镜像

> **日期：** 2026-10-24
> **产品线：** 共用（Edge 开发板环境）
> **环境：** lubancat RK3588 / Ubuntu 22.04.5 / 内核 6.1.84 / ROOT 在 512G 的 `mmcblk1`（SD）

## 现象

官方镜像只有 2G 多，但板上跑了一段时间后 `df` 显示根分区 476G、已用 36G。
直接 `dd if=/dev/mmcblk1` 会得到 476G 镜像，既没法放进另一张卡做「环境复制」，传输也不现实。
目标：产出十几到二十几 G 的可烧录镜像，烧到 ≥64G 新卡后功能与当前系统一致。

## 根因

1. **镜像大小由分区表决定**，不是由已用数据决定；厂商镜像把 p2 铺满整卡，所以"看起来"是 512G。
2. **不能在运行中缩小根文件系统**：`resize2fs` 离线缩小要求先 `e2fsck -f` 且分区未挂载，而当前系统就跑在这块卡上。
3. **引导不在分区里**：RK3588 的 `idbloader`/`u-boot.itb` 位于分区表之前的裸扇区（本机 sector 64 起为 `RKNS`，sector 16384 起为 FIT `d00dfeed`），只拷分区会得到不能引导的镜像。
4. **根设备写死在 U-Boot 脚本里**：`/boot/firmware/boot.cmd` 中 `setenv bootargs "... root=/dev/mmcblk${devnum}p2"`，即目标卡必须仍是 `mmcblk1`（eMMC 占 `mmcblk0` 时的 SD 槽位）。

## 正确做法

用「稀疏镜像文件 + 重建 GPT + 拷裸引导区 + 分区格式化 + rsync 整机」离线拼镜像，**完全不改动源盘**（不用重启、不用从 eMMC 引导）。脚本见 `/home/cat/images/build-image.sh`，排除清单 `/home/cat/images/rsync-exclude.txt`。

关键步骤：

```bash
truncate -s 24G img                                   # 目标大小
sgdisk --zap-all img                                  # 重建 GPT
sgdisk -n 1:32768:+1048576 -t 1:BC13C2FF-... -u 1:<原p1-uuid> -c 1:primary img
sgdisk -n 2:1081344:<last-34> -t 2:0FC63DAF-... -u 2:<原p2-uuid> -c 2:primary img
dd if=/dev/mmcblk1 of=img bs=512 skip=34 seek=34 \
   count=$((32768-34)) conv=notrunc                  # 只拷 34..32767，别覆盖新 GPT
losetup -fP img && mkfs.vfat -F32 -n system-boot ${LOOP}p1
mkfs.ext4 -F -L writable -m 1 ${LOOP}p2
rsync -aHAX --numeric-ids --exclude-from=rsync-exclude.txt / /mnt/img/
rsync -aHAX --numeric-ids /boot/firmware/ /mnt/img/boot/firmware/
fallocate -l 2G /mnt/img/swapfile && mkswap /mnt/img/swapfile && : > /mnt/img/etc/machine-id
e2fsck -fn ${LOOP}p2
```

本机实测：排除 IDE server/缓存/日志/包缓存/大 snap 后，内容 19.5 GB → **24 GiB 镜像**（`df` 剩 2.4G），`pigz -6` 压缩后 **7.3G**。

新卡首次启动后必须扩分区（`x-systemd.growfs` 只扩文件系统、不扩分区）：

```bash
sudo growpart /dev/mmcblk1 2 && sudo resize2fs /dev/mmcblk1p2
```

## 坑

- **只 `dd` 前 16MiB 会把旧 GPT 一起带进来**（`last-lba` 指向 476G，分区越界）→ 必须先用 `sgdisk` 写新表，再从 sector 34 开始拷裸区，最后 `sgdisk -v` 验证。
- **排除项只能写进 rsync，别删源**：`.trae-cn-server`/`.cursor-server` 正被当前 IDE 使用（5.3G+2.9G），删了会当场断会话。
- 同一张卡读+写，实测 rsync 只有 ~19 MB/s，19.5G 约 20 分钟；不要误以为是卡死了。
- 清空 `/etc/machine-id` 让 systemd 首启重新生成；SSH host key 保留可让旧 `known_hosts` 继续可用，但两块板会共用同一份主机密钥。
- 两块板同名 `lubancat` 在同一局域网会有 mDNS 冲突，需要时改 `/etc/hostname`。
- fstab 用 `LABEL=`，所以分区 UUID 变不变都不影响启动；但 `root=` 是 `/dev/mmcblk1p2`，换板子/换槽位（例如插 USB 读卡器启动）会找不到根。

## 相关路径

- `/home/cat/images/build-image.sh`、`/home/cat/images/rsync-exclude.txt`（构建脚本与排除清单）
- `/boot/firmware/boot.cmd`（`root=/dev/mmcblk${devnum}p2`）
- `/etc/fstab`（`LABEL=writable` + `x-systemd.growfs`）
