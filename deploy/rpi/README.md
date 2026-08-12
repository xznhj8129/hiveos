# MPFC Raspberry Pi appliance

This directory builds the MPFC companion-computer appliance for a Raspberry Pi
Zero 2 W class target and boots the same raw image under QEMU for development.

The image contains MPFC, OCCID, HiveLink, one Python environment, a loopback-only
Mosquitto broker, and the MPFC systemd runtime. It is not part of Sigma's host
installation and does not require Sigma to build or run.

## Configuration

Standalone defaults live in:

```text
deploy/rpi/defaults.env
```

They define the appliance node identity, control-node address, guest address,
network prefix, HiveLink port, bridge/tap names, and physical/VM MAVLink
connections. A deployment manager may override those variables in the
environment before invoking `build-image` or `pi-vm`.

Runtime configuration is generated rather than checked into the rootfs tree:

- `install-rootfs` writes `/etc/mpfc/runtime.env` and the QEMU Ethernet profile
  from the resolved build settings;
- `pi-vm` injects VM-only overrides on the removable `MPFC_CONFIG` disk;
- `run-mpfc` renders the final `/run/mpfc/config.yaml`, including node identity,
  HiveLink addressing, and MAVSDK connection, immediately before MPFC starts.

## Host prerequisites

On Debian, Ubuntu, or Mint:

```bash
sudo apt install \
  git curl rsync xz-utils parted e2fsprogs \
  qemu-system-arm qemu-user-static binfmt-support \
  dosfstools mtools iproute2
```

Keep the three image source repositories as siblings when using MPFC directly:

```text
~/opt/
  mpfc/
  occid/
  hivelink/
```

## Build the image

```bash
sudo ./deploy/rpi/build-image
```

The builder uses a pinned Raspberry Pi OS Lite 64-bit base image and verifies
its SHA256. Outputs are written under `deploy/rpi/dist/`:

```text
mpfc-rpi-zero2w.img
mpfc-rpi-zero2w.img.sha256
mpfc-rpi-zero2w.img.manifest
mpfc-rpi-zero2w.img.kernel8.img
mpfc-rpi-zero2w.img.raspi3ap.dtb
```

The raw `.img` is the physical SD-card artifact. The kernel and DTB sidecars are
only for QEMU direct boot.

When `MPFC_PI_SSH_KEY` names a private key and the matching `.pub` file exists,
the builder installs that public key into `/home/mpfc/.ssh/authorized_keys` in
the raw image. Sigmac3 supplies its configured MPFC SSH identity when it builds
the appliance. A standalone build without a key is allowed but emits a warning.

The manifest records the Raspberry Pi OS base hash, MPFC/OCCID/HiveLink source
revisions, node names, IP addresses, prefix, HiveLink port, physical/VM MAVLink
defaults, and the installed SSH public-key hash when one was supplied.

A deployment manager can therefore detect that an existing image is stale
instead of silently running an appliance built from different sources or
network settings.

## Physical Pi

Write the image with Raspberry Pi Imager or another raw image writer. For
example:

```bash
sudo dd if=deploy/rpi/dist/mpfc-rpi-zero2w.img \
  of=/dev/sdX bs=4M status=progress conv=fsync
```

After booting the physical Pi, assign the actual flight-controller serial port:

```bash
sudo /opt/mpfc/deploy/rpi/configure-fc /dev/ttyAMA0 921600
```

or:

```bash
sudo /opt/mpfc/deploy/rpi/configure-fc /dev/ttyUSB0 460800
```

`configure-fc` changes only `MPFC_MAVLINK_CONNECTION`. It preserves the node
identity and network configuration built into `/etc/mpfc/runtime.env`.

Physical network provisioning remains a deployment concern. The QEMU Ethernet
profile generated into the image exists for the virtual test article and may be
replaced by the real vehicle's Ethernet or Wi-Fi configuration.

## Virtual appliance

Start the same image in the foreground:

```bash
./deploy/rpi/pi-vm up
```

or in the background:

```bash
./deploy/rpi/pi-vm start
```

`pi-vm` creates the configured Linux bridge and TAP device. The guest is an
independently addressed IP node, not a localhost port-forward shim. QEMU boots
the raw image in snapshot mode, so VM writes are discarded on stop.

The `MPFC_CONFIG` disk supplies the configured VM MAVLink endpoint, node/link
settings, and SSH public key. The base image remains the same physical artifact.

Useful commands:

```bash
./deploy/rpi/pi-vm ssh
./deploy/rpi/pi-vm logs
./deploy/rpi/pi-vm deploy
./deploy/rpi/pi-vm stop
```

`deploy` rsyncs the current MPFC checkout and sibling OCCID/HiveLink checkouts
into a running guest for fast Python iteration and restarts MPFC. Rebuild the
image when validating the actual appliance artifact.

Use another SSH identity with:

```bash
MPFC_PI_SSH_KEY=$HOME/.ssh/another_key ./deploy/rpi/pi-vm start
```

## PX4 testing

PX4 SITL is a test peer for MPFC/MAVSDK. It is deliberately not installed or
started by the MPFC image itself. A higher-level development manager may start
PX4 on the control host and point its onboard MAVLink stream at the appliance's
configured VM endpoint.

No MAVLink relay is required for the bridged QEMU setup.

## Guest services

```text
ssh.service
mosquitto.service
mpfc-runtime-config.service
mpfc.service
```

Mosquitto listens only on loopback and remains private MPFC node-local IPC.

Useful diagnostics on the guest:

```bash
journalctl -fu mpfc.service
mosquitto_sub -h 127.0.0.1 -p 1883 -v -t 'mpfc/#'
```

## Validation boundary

The VM validates the companion-computer deployment boundary: ARM64 packages,
Python/MAVSDK behavior, service startup, private local IPC, OCCID, HiveLink,
independent IP networking, and PX4 interaction.

It does not validate Zero 2 W GPIO, RF behavior, electrical behavior, or timing
characteristics of physical serial hardware.
