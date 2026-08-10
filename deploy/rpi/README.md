# MPFC Raspberry Pi appliance

This directory defines the reference MPFC companion computer appliance for a
Raspberry Pi Zero 2 W class target.

The goal is deliberately simple:

1. Build one Raspberry Pi OS Lite 64-bit image that is ready to burn to an SD
   card and run on a real Zero 2 W.
2. Boot that same raw image immediately in QEMU as a disposable 4-core,
   512 MiB ARM64 Pi-class VM for PX4/MPFC development.
3. Keep the source-repository boundary to `mpfc`, `occid`, and `hivelink`.

The physical and virtual machines use the same filesystem, packages, services,
MPFC checkout, OCCID checkout, HiveLink checkout, Python environment, Mosquitto
configuration, and systemd units. Only the FC transport is overridden:

```text
physical: serial:///dev/<deployment-assigned-tty>:<baud>
VM:       udp://:14540
```

## Host prerequisites

On a Debian/Ubuntu/Mint development machine:

```bash
sudo apt install \
  git curl rsync xz-utils parted e2fsprogs \
  qemu-system-arm qemu-user-static binfmt-support \
  dosfstools mtools
```

Keep the three source repositories as siblings:

```text
~/opt/
  mpfc/
  occid/
  hivelink/
```

Alternate OCCID and HiveLink paths can be passed to `build-image`.

## Build the SD/VM image

From the MPFC checkout:

```bash
sudo ./deploy/rpi/build-image
```

The builder is pinned to Raspberry Pi OS Lite 64-bit from 18 Jun 2026 and
checks the official compressed-image SHA256 before modifying it. The output is:

```text
deploy/rpi/dist/
  mpfc-rpi-zero2w.img
  mpfc-rpi-zero2w.img.sha256
  mpfc-rpi-zero2w.img.manifest
  mpfc-rpi-zero2w.img.kernel8.img
  mpfc-rpi-zero2w.img.raspi3ap.dtb
```

The two sidecars are used only for QEMU direct boot. The `.img` itself is the
physical SD-card artifact.

Build from a previously downloaded image instead:

```bash
sudo ./deploy/rpi/build-image \
  --base-image /path/to/raspios-lite-arm64.img.xz \
  --base-sha256 <sha256>
```

Add `--compress` when you also want an `.img.xz` distribution artifact.

The manifest records the base image hash and the MPFC/OCCID/HiveLink revisions
used for the build.

## Burn a real Pi

Use Raspberry Pi Imager, or write the raw image directly:

```bash
sudo dd if=deploy/rpi/dist/mpfc-rpi-zero2w.img \
  of=/dev/sdX bs=4M status=progress conv=fsync
```

The default FC endpoint in the image is `/dev/serial0` at 921600 baud. The
normal deployment procedure should explicitly assign the real FC tty:

```bash
sudo /opt/mpfc/deploy/rpi/configure-fc /dev/ttyAMA0 921600
```

or, for a USB/UART adapter:

```bash
sudo /opt/mpfc/deploy/rpi/configure-fc /dev/ttyUSB0 460800
```

`configure-fc` writes `/etc/mpfc/runtime.env`, adds the service user to
`dialout`, and restarts MPFC. This is the integration point for the broader
hardware deployment script: pass it the tty that script already selected.

## Run the virtual Pi

Start a disposable VM in the foreground:

```bash
./deploy/rpi/pi-vm up
```

Or daemonize it:

```bash
./deploy/rpi/pi-vm start
```

The VM uses QEMU's `raspi3ap` machine, which matches the Zero 2 W's useful test
budget closely: four Cortex-A53 cores and 512 MiB RAM. It boots the exact same
raw SD image with QEMU snapshot mode, so guest writes are discarded when the VM
stops.

The helper creates a tiny removable FAT volume labelled `MPFC_CONFIG`. That
volume overrides only:

```text
MPFC_MAVLINK_CONNECTION=udp://:14540
```

The base image therefore remains a real-hardware image. There is no separate
"VM image" to drift out of sync.

Host mappings are:

```text
127.0.0.1:2222/tcp  -> guest :22/tcp    SSH
127.0.0.1:14540/udp -> guest :14540/udp PX4 companion MAVLink
```

Point PX4 SITL's companion output at host UDP port 14540. Inside the VM,
`mavsdk_interface` receives it exactly as an external companion endpoint.

Useful commands:

```bash
./deploy/rpi/pi-vm ssh
./deploy/rpi/pi-vm logs
./deploy/rpi/pi-vm deploy
./deploy/rpi/pi-vm stop
```

`deploy` rsyncs the current MPFC checkout and sibling OCCID/HiveLink checkouts
into the running guest and restarts MPFC. Rebuilding the SD image is therefore
not required for ordinary Python development.

The VM injects `$HOME/.ssh/id_ed25519.pub` by default. Override it with:

```bash
MPFC_PI_SSH_KEY=$HOME/.ssh/another_key ./deploy/rpi/pi-vm start
```

## Guest layout

```text
/opt/mpfc
/opt/occid
/opt/hivelink
/opt/mpfc/.venv
/etc/mpfc/runtime.env
/var/log/mpfc/hivebus.log
```

Services enabled in the image:

```text
ssh.service
mosquitto.service
mpfc-runtime-config.service
mpfc.service
```

Watch MPFC directly on either real or virtual hardware:

```bash
journalctl -fu mpfc.service
```

Watch the local MQTT/OCCID bus:

```bash
mosquitto_sub -h 127.0.0.1 -p 1883 -v -t 'mpfc/#'
```

## Known boundary

This VM is intended to prove the companion-computer deployment: ARM64 package
availability, Python/MAVSDK behavior, 512 MiB memory pressure, service startup,
MQTT, OCCID, HiveLink, networking, and PX4 interaction. It does not pretend to
validate Zero 2 W GPIO, Wi-Fi, Bluetooth, electrical behavior, or timing of
physical serial hardware.
