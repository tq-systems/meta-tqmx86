# Openembedded/Yocto hardware support layer for TQ-Systems x86 COMs

This README file contains information on the contents of the meta-tqmx86 layer.
This layer provides support for TQ-Systems COMs (Computer-on-Modules) with x86
CPU.


## Dependencies

This layer depends on a number of base layers, which are listed in the
following.

The correct branch matching the Yocto version must be checked out for each
layer. The supported Yocto version is "kirkstone". Historical branches for
building against older Yocto versions like "hardknott" and "gatesgarth" can
also be found in this repository.

- poky

  URL: https://git.yoctoproject.org/cgit.cgi/poky/

- meta-intel

  URL: http://git.yoctoproject.org/cgit/cgit.cgi/meta-intel


## Patches

Please submit patches against the *meta-tqmx86* layer to the TQ-Systems
Linux team (linux@ew.tq-group.com) or use Github's collaboration features.


## Build setup

In order to use this layer you need to make the build system aware of it.

Assuming the all layers exist in a subdir `sources` at the top-level of your
Yocto build tree, you can add it to the build system by adding the
location of the *meta-tqmx86* layer to `bblayers.conf` along with any
other layers needed, e.g.:

```
BBLAYERS ?= " \
  ${BSPDIR}/sources/poky/meta \
  ${BSPDIR}/sources/poky/meta-poky \
  ${BSPDIR}/sources/meta-intel \
  ${BSPDIR}/sources/meta-tqmx86 \
"
```

## Supported hardware

All TQ-Systems x86 hardware listed in the following is supported by the
`MACHINE` configuration "intel-corei7-64-tqmx86". The `core-image-base` and
the `DISTRO` "poky" are the recommended starting point for customization of the
BSP.

For many COMs, variants with alternative CPUs are available, and different
mainboards with the same form factor can be used. The boards listed in the
following table have been used for testing the Yocto BSP.

|     | COM                | Mainboard  | CPU                          | Form factor                |
|-----|--------------------|---------------|------------------------------|----------------------------|
| `b` | TQMx50UC           |            | Intel Core (5th generation)  | COM Express Compact Type 6 |
| `b` | TQMx60EB           |            | Intel Core (6th generation)  | COM Express Basic Type 6   |
| `b` | TQMx70EB           |            | Intel Core (7th generation)  | COM Express Basic Type 6   |
| `b` | TQMx80UC           |            | Intel Core (8th generation)  | COM Express Compact Type 6 |
| `y` | TQMx110EB          | MB-COME6-4 | Intel Core (11th generation) | COM Express Basic Type 6   |
| `p` | TQMx120EB          |            | Intel Core (12th generation) | COM Express Basic Type 6   |
| `p` | TQMx130EB          |            | Intel Core (13th generation) | COM Express Basic Type 6   |
| `b` | TQMxE38C           |            | Intel Atom E3800             | COM Express Compact Type 6 |
| `b` | TQMxE38M           |            | Intel Atom E3800             | COM Express Mini Type 10   |
| `b` | TQMxE39C1/2        |            | Intel Atom E3900             | COM Express Compact Type 6 |
| `b` | TQMxE39M           |            | Intel Atom E3900             | COM Express Mini Type 10   |
| `b` | TQMxE39S           |            | Intel Atom E3900             | SMARC 2.0/2.1              |
| `b` | TQMxE40C1/2        |            | Intel Atom x6000             | COM Express Compact Type 6 |
| `b` | TQMxE40M           |            | Intel Atom x6000             | COM Express Mini Type 10   |
| `y` | TQMxE40S           | MB-SMARC-3 | Intel Atom x6000             | SMARC 2.1                  |
| `p` | TQMxE41S           |            | Intel Atom x7000E            | SMARC 2.1                  |

|     | Support status    |
|-----|-------------------|
| `y` | supported         |
| `b` | build tested only |
| `p` | prerelease        |

## Known issues

- linux-intel:
  - The kernel contains calls to `trace_printk()`, which should only be used
    for debugging. The kernel warns about this during boot.
  - The both-edges IRQ trigger of the TQMx86 GPIO driver will report multiple
    falling or rising edge events in a row instead of alternating between the
    two when the edges are received faster than they can be reported
- TQMx110EB:
  - DisplayPort DP3 of the MB-COME6-4 does not work
  - Microphone input of MB-COME6-4 does not work
  - A warning with backtrace from the i915 driver is printed to the kernel log
    when booting with LVDS enabled
- TQMxE40S:
  - There is no driver support for the eSPI UART controller of the MB-SMARC-3
    (SER4/SER5)
