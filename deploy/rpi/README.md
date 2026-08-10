# MPFC Raspberry Pi appliance

This directory defines the reference MPFC companion-computer appliance for a
Raspberry Pi Zero 2 W class target.

The appliance has two jobs:

1. Build one Raspberry Pi OS Lite 64-bit image that is ready to write to an SD
   card and run on a real Zero 2 W.
2. Boot that same raw image under QEMU as an independently addressed ARM64 node
   for MPFC/OCCID/HiveLink/PX4 development.

The image contains the MPFC, OCCID, and HiveLink source trees, one Python
environment, a loopback-only Mosquitto broker, and the MPFC systemd runtime.
The physical and virtual forms differ only in deployment-specific runtime
configuration such as the FC connection and network provisioning.

Reference FC connections:

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
  dosfstools mtools iproute2
```

Keep the source repositories as siblings:

```text
~/opt/
  mpfc/
  occid/
  hivelink/
```

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

Add `--compress` when an `.img.xz` distribution artifact is also wanted.

The manifest records the base image hash and the MPFC/OCCID/HiveLink revisions
used for the build.

## Burn a real Pi

Use Raspberry Pi Imager, or write the raw image directly:

```bash
sudo dd if=deploy/rpi/dist/mpfc-rpi-zero2w.img \
  of=/dev/sdX bs=4M status=progress conv=fsync
```

The default physical FC endpoint in the image is `/dev/serial0` at 921600 baud.
The deployment procedure should explicitly assign the real FC tty:

```bash
sudo /opt/mpfc/deploy/rpi/configure-fc /dev/ttyAMA0 921600
```

or, for a USB/UART adapter:

```bash
sudo /opt/mpfc/deploy/rpi/configure-fc /dev/ttyUSB0 460800
```

`configure-fc` writes `/etc/mpfc/runtime.env`, adds the service user to
`dialout`, and restarts MPFC.

Network configuration is a deployment concern. The image includes the reference
QEMU Ethernet profile used below; a physical installation may replace that
profile with its actual Ethernet/Wi-Fi configuration without changing MPFC,
OCCID, or HiveLink semantics.

## Run the virtual Pi

The reference development network is:

```text
control host / cfbr0   192.168.0.220/24
MPFC guest             192.168.0.230/24
```

`pi-vm` creates a Linux bridge and TAP device, so it requires sudo capability
for network-device setup. The guest is a real IP peer; there are no localhost
SSH or MAVLink port-forward shims.

Start a disposable VM in the foreground:

```bash
./deploy/rpi/pi-vm up
```

Or daemonize it:

```bash
./deploy/rpi/pi-vm start
```

The VM uses QEMU's `raspi3ap` machine with four Cortex-A53 cores and 512 MiB
RAM. It boots the exact same raw SD image with QEMU snapshot mode, so guest
writes are discarded when the VM stops.

The helper creates a tiny removable FAT volume labelled `MPFC_CONFIG`. For the
VM it injects:

```text
MPFC_MAVLINK_CONNECTION=udp://:14540
```

and the selected SSH public key. The base image therefore remains the physical
artifact rather than having a separate VM fork.

The two independent server-to-Pi paths are:

```text
Sigma/control 192.168.0.220:5555
    -> HiveLink/OCCID UDP
    -> MPFC Pi 192.168.0.230:5555

PX4 SITL onboard MAVLink
    -> cfbr0 broadcast, remote UDP 14540
    -> MAVSDK inside MPFC Pi
```

With PX4 POSIX SITL, setting `PX4_NET_INTERFACE=cfbr0` causes its normal onboard
MAVLink instance to bind the bridge interface and enable broadcast. No MAVLink
relay is required.

Useful commands:

```bash
./deploy/rpi/pi-vm ssh
./deploy/rpi/pi-vm logs
./deploy/rpi/pi-vm deploy
./deploy/rpi/pi-vm stop
```

`deploy` rsyncs the current MPFC checkout and sibling OCCID/HiveLink checkouts
into the running guest, reinstalls the editable HiveLink package, and restarts
MPFC. Rebuilding the SD image is not required for ordinary Python iteration.

The VM injects `$HOME/.ssh/id_ed25519.pub` by default. Use another identity with:

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

Mosquitto listens only on `127.0.0.1:1883`. It is MPFC's private node-local
IPC and is not the external autonomous-node API.

Watch MPFC directly on either real or virtual hardware:

```bash
journalctl -fu mpfc.service
```

Watch the private node-local bus:

```bash
mosquitto_sub -h 127.0.0.1 -p 1883 -v -t 'mpfc/#'
```

## Validation boundary

The VM is intended to validate the companion-computer deployment boundary:
ARM64 package availability, Python/MAVSDK behavior, 512 MiB memory pressure,
service startup, private local IPC, OCCID, HiveLink, independent IP networking,
and PX4 interaction.

It does not validate Zero 2 W GPIO, Wi-Fi/Bluetooth RF behavior, electrical
behavior, or timing characteristics of physical serial hardware.