SUMMARY = "TQMx86 kernel drivers"
LICENSE = "GPLv2"
LIC_FILES_CHKSUM = "file://COPYING;md5=12f884d2ae1ff87c09e5b7ccc2c4ca7e"

inherit module

SRC_URI = "file://Makefile \
           file://gpio-tqmx86.c \
           file://i2c-machxo2.c \
           file://i2c-machxo2.h \
           file://i2c-wishbone.c \
           file://tqmx86.c \
           file://tqmx86_wdt.c \
           file://COPYING \
          "

S = "${WORKDIR}"

# The inherit of module.bbclass will automatically name module packages with
# "kernel-module-" prefix as required by the oe-core build environment.

RPROVIDES_${PN} += "kernel-module-tqmx86 kernel-module-gpio-tqmx86 kernel-module-tqmx86_wdt kernel-module-i2c-machxo2 kernel-module-i2c-wishbone"
