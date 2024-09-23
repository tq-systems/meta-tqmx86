# Changelog

All notable changes to this project will be documented in this file.
Releases are named with the following scheme:

`<Yocto Project version name>.<TQ module family>.BSP.SW.<version number>`

[[_TOC_]]

## Next Release

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
