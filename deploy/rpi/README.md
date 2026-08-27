# MPFC Raspberry Pi appliance

This directory builds the MPFC companion-computer appliance for a Raspberry Pi
Zero 2 W class target. The same raw image can also be booted under QEMU for ARM
qualification; normal MPFC development uses the x86 KVM runtime managed by
Sigmac3.

The image contains MPFC, installed OCCID and HiveLink packages, a Python runtime,
loopback-only Mosquitto, SSH, and the MPFC systemd runtime. It is independent of
Sigma.

## Build model

The appliance build is layered so normal development does not repeatedly run
package installation under ARM emulation.

1. A pinned Raspberry Pi OS Lite ARM64 image is expanded and provisioned with
   the small system/runtime package set. This prepared OS layer is cached by the
   base-image hash, image size, and `install-rootfs` content.
2. MPFC/HiveLink Python dependencies that have ARM64 wheels are resolved and
   installed by host x86 Python/pip using explicit target platform, Python, and
   ABI selectors. The resulting target `site-packages` layer is cached by the
   resolved requirements inputs.
3. `crcmod` is supplied by the Debian ARM64 `python3-crcmod` package because it
   contains a target-native extension.
4. OCCID and HiveLink are installed into the target virtualenv as normal Python
   packages from the selected source checkouts. Their source trees are not
   copied into the appliance and no runtime path override is used.
5. MPFC application source, service files, identity, password, network settings,
   and runtime configuration are host-side overlays onto the prepared image.
6. QEMU system emulation is optional ARM runtime qualification, not routine
   Python dependency installation or normal development.

`MPFC_BUILD_CACHE` may point at a persistent cache outside the source checkout.
Sigmac3 defaults this to `~/.cache/sigmac3/mpfc`, so deleting/recloning the MPFC
deployment root does not gratuitously repeat unchanged dependency work.

## Configuration

Standalone defaults live in:

```text
deploy/rpi/defaults.env
```

They define node identity, control/guest addresses, network prefix, HiveLink
port, bridge/tap names, physical/VM MAVLink connections, and the appliance
password.

Default login:

```text
user:     mpfc
password: mpfc
```

Override `MPFC_PI_PASSWORD` before building/booting when another appliance
password is desired. Password authentication is deliberate; the appliance does
not depend on a developer's personal SSH key.

At runtime:

- `/etc/mpfc/runtime.env` contains physical/default appliance settings;
- QEMU supplies VM-only settings on the removable `MPFC_CONFIG` disk;
- `run-mpfc` renders `/run/mpfc/config.yaml` immediately before MPFC starts.

OCCID and HiveLink are ordinary installed packages in `/opt/mpfc/.venv`. There
is no `OCCID_PATH`, `HIVELINK_PATH`, source `.pth`, `/opt/occid`, or
`/opt/hivelink` runtime import mechanism.

The image is provisioned before first boot. Raspberry Pi OS `userconfig.service`
and the unused `systemd-networkd-wait-online.service` are masked in the prepared
system layer.

## Host prerequisites

On Debian, Ubuntu, or Mint:

```bash
sudo apt update
sudo apt install -y \
  git curl python3 python3-venv python3-pip openssl \
  rsync xz-utils parted e2fsprogs \
  qemu-user-static binfmt-support qemu-system-arm \
  dosfstools mtools device-tree-compiler \
  iproute2 openssh-client sshpass iputils-ping
```

The first prepared-system cache miss uses `qemu-aarch64-static` only for the
small target OS provisioning step. ARM `pip` is not used.

## Build the image

Keep MPFC, OCCID, and HiveLink as sibling checkouts, then run:

```bash
sudo ./deploy/rpi/build-image
```

The OCCID/HiveLink checkout paths are build inputs only. `--occid-path` and
`--hivelink-path` may select other checkouts without creating runtime path
coupling in the resulting image.

Outputs under `deploy/rpi/dist/`:

```text
mpfc-rpi-zero2w.img
mpfc-rpi-zero2w.img.sha256
mpfc-rpi-zero2w.img.manifest
mpfc-rpi-zero2w.img.kernel8.img
mpfc-rpi-zero2w.img.raspi3ap.dtb
```

The manifest records the base image, prepared-system cache key, Python-layer
cache key, target Python version, source revisions, node/network settings,
MAVLink defaults, and password-auth login mode.

A source-only MPFC rebuild reuses the prepared system/Python layers when their
inputs have not changed. OCCID or HiveLink changes are installed through normal
Python packaging during the image build.

## Physical Pi

Write the raw image with Raspberry Pi Imager or another raw image writer, for
example:

```bash
sudo dd if=deploy/rpi/dist/mpfc-rpi-zero2w.img \
  of=/dev/sdX bs=4M status=progress conv=fsync
```

After boot, configure the actual FC serial endpoint:

```bash
sudo /opt/mpfc/deploy/rpi/configure-fc /dev/ttyAMA0 921600
```

or:

```bash
sudo /opt/mpfc/deploy/rpi/configure-fc /dev/ttyUSB0 460800
```

`configure-fc` changes only `MPFC_MAVLINK_CONNECTION`.

## Virtual appliance

Foreground:

```bash
./deploy/rpi/pi-vm up
```

Background:

```bash
./deploy/rpi/pi-vm start
```

The host bridge owns only its `/32` control address and one explicit `/32` route
to the guest, so it never claims the physical LAN prefix.

QEMU uses a disposable DTB copy that disables the BCM2835 PM/watchdog provider
which current Raspberry Pi kernels probe but QEMU's Pi 3 PM/ASB model does not
fully implement. The physical image and original DTB remain unchanged.

The VM uses snapshot mode, so guest writes are discarded on stop. The serial
console is retained at `deploy/rpi/.vm/console.log`.

Useful commands:

```bash
./deploy/rpi/pi-vm ssh
./deploy/rpi/pi-vm logs
./deploy/rpi/pi-vm deploy
./deploy/rpi/pi-vm stop
```

`ssh`, `logs`, and `deploy` use `MPFC_PI_PASSWORD` through `sshpass`; they do not
read personal SSH keys.

`deploy` is only for MPFC application-source iteration. It rsyncs `/opt/mpfc`,
checks the installed OCCID contract, and restarts MPFC. OCCID, HiveLink, or other
Python dependency changes require an image rebuild so the installed environment
remains authoritative.

## PX4 testing

PX4 SITL is a test peer for MPFC/MAVSDK and is not installed in the appliance.
A development manager may launch PX4 on the control host while QEMU boots and
then validate MPFC against the ready peer.

## Guest services

```text
ssh.service
mosquitto.service
mpfc-runtime-config.service
mpfc.service
```

Mosquitto listens only on loopback and remains private MPFC node-local IPC.

Useful diagnostics:

```bash
journalctl -fu mpfc.service
mosquitto_sub -h 127.0.0.1 -p 1883 -v -t 'mpfc/#'
```

## Validation boundary

The VM validates the ARM64 userspace, cross-staged Python/MAVSDK runtime,
systemd services, private local IPC, installed OCCID/HiveLink packages,
independent IP networking, and PX4 interaction. It does not validate physical
Zero 2 W GPIO, RF, electrical behavior, or serial timing.
