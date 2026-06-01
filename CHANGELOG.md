# Changelog

All notable changes to this project will be documented in this file.
Releases are named with the following scheme:

`<Yocto Project version name>.<TQ module family>.BSP.SW.<version number>`

[[_TOC_]]

## Next Release

### Changed

- Switched from Yocto scarthgap to wrynose
- Switched from linux-yocto(-rt) 6.12.y to 6.18.y

  The specific patch release of 6.18.y is defined by the used revision
  of openembedded-core.
- Replaced unmaintained cpufrequtils package with cpupower
- Switched from busybox to full coreutils

## scarthgap.TQMx86.BSP.SW.0003

### Added

- Added recipes for linux-yocto(-rt) 6.12.63
- Added support for TQMxCU1-HPCM (as well as the planned TQMxCU2-HPCM)
- Added support for TQMxE41M
- Added support for the secondary (MachXO2) I2C controller of various TQMx86
  COMs
- Added ptn3460-edid-tool to program EDID information on PTN3460 eDP to LVDS bridges
  found on many TQMx86 COMs
- Added two distro configs "pretzel.conf" and "pretzel-rt.conf".

  "pretzel" is the the new TQ-x86 distro and "pretzel-rt" extends
  it with a realtime kernel (`linux-yocto-rt`).

### Fixed

- Fixed order of MACHINEOVERRIDES

  `tqmx86` is now more specific than `corei7-64-intel-common` and
  `intel-x86-common`, allowing to use it to override settings from the
  meta-intel machine includes
- Added `usbutils` to provide `usb-devices`. This also required to switch
  `find` from busybox-find to GNU-find by adding `findutils`.

### Changed

- Do not install contents of ESP to root filesystem by default

  `/boot` is the mountpoint for the ESP. To avoid confusion when looking
  at the filesystem without this mount, do not install the kernel or
  bootloader components to this directory.
- Generate compressed rootfs image and archive

  By default, images are now generated in the following formats:

  - wic.zst (including a wic.bmap for bmaptool)
  - tar.zst

  The used compression algorithm can be adjusted using the
  `TQ_IMAGE_COMPRESSION` variable.

### Removed

- Removed linux-yocto(-rt) 6.6 recipes

## scarthgap.TQMx86.BSP.SW.0002

### Added

- Added support for linux-yocto and linux-yocto-rt 6.6 kernels

  linux-yocto 6.6.50 is the default kernel for the TQMx86 machine now.

### Fixed

- Fixed a race condition in interrupt handling of the TQMxE40x CAN controller,
  which could result in a stuck TX queue under high load

### Changed

- The kernel configuration is now based on the Yocto configuration template
  ("KMACHINE") `intel-x86-64`, which provides better defaults for modules with
  Atom CPUs in addition to the Core i7-based modules.
- The `intel-corei7-64-tqmx86` machine has been renamed to `intel-x86-64-tqmx86`
  to reflect the new KMACHINE.

### Removed

- Removed support for the linux-intel 5.15 kernel

## kirkstone.TQMx86.BSP.SW.0001

This is the first Linux BSP release for TQMx86 modules based on
Yocto kirkstone.

It uses linux-intel 5.15.129 as its kernel, extended with driver support for
TQ-specific hardware components.
