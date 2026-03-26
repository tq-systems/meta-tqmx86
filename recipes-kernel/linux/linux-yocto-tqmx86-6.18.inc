FILESEXTRAPATHS:prepend:tqmx86 := "${THISDIR}/linux-yocto-tqmx86-6.18:"

SRC_URI:append:tqmx86 = " \
    file://0001-mfd-tqmx86-Add-board-definitions-for-TQMxCU1-HPCM-an.patch \
    file://0002-mfd-tqmx86-add-board-definition-for-TQMxE41M.patch \
    file://0003-i2c-ocores-increase-poll-timeout-to-total-transfer-t.patch \
    file://0004-i2c-ocores-respect-adapter-timeout-in-IRQ-mode.patch \
    file://0005-i2c-machxo2-new-driver.patch \
    file://0006-mfd-tqmx86-refactor-I2C-setup.patch \
    file://0007-mfd-tqmx86-add-detection-for-secondary-I2C-controlle.patch \
    file://0008-mfd-tqmx86-add-detection-for-MachXO2-I2C-controller.patch \
    file://0009-gpio-tqmx86-introduce-platform-data-struct.patch \
    file://0010-mfd-tqmx86-update-TQMX86_IOSIZE_GPIO.patch \
    file://0011-mfd-tqmx86-configure-board-specific-GPIO-platform-da.patch \
    file://0012-gpio-tqmx86-prepare-support-for-variants-with-more-G.patch \
    file://0013-gpio-tqmx86-add-support-for-up-to-16-GPIOs.patch \
    \
    file://nct7802.cfg \
    file://tqmx86.cfg \
"

KMACHINE:intel-x86-64-tqmx86 = "intel-x86-64"
