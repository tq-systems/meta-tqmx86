FILESEXTRAPATHS:prepend:tqmx86 := "${THISDIR}/linux-yocto-tqmx86-6.12:"

SRC_URI:append:tqmx86 = " \
    file://0001-mfd-tqmx86-Add-board-definitions-for-TQMx120UC-TQMx1.patch \
    file://0002-mfd-tqmx86-Improve-gpio_irq-module-parameter-descrip.patch \
    file://0003-mfd-tqmx86-Refactor-GPIO-IRQ-setup.patch \
    file://0004-mfd-tqmx86-Make-IRQ-setup-errors-non-fatal.patch \
    file://0005-mfd-tqmx86-Add-I2C-IRQ-support.patch \
    file://0006-gpio-tqmx86-add-macros-for-interrupt-configuration.patch \
    file://0007-gpio-tqmx86-consistently-refer-to-IRQs-by-hwirq-numb.patch \
    file://0008-gpio-tqmx86-use-cleanup-guards-for-spinlock.patch \
    file://0009-gpio-tqmx86-introduce-tqmx86_gpio_clrsetbits-helper.patch \
    file://0010-gpio-tqmx86-add-support-for-changing-GPIO-directions.patch \
    file://0011-mfd-tqmx86-add-board-definitions-for-TQMxCU1-HPCM-an.patch \
    file://0012-i2c-ocores-increase-poll-timeout-to-total-transfer-t.patch \
    file://0013-i2c-ocores-respect-adapter-timeout-in-IRQ-mode.patch \
    file://0014-i2c-machxo2-new-driver.patch \
    file://0015-mfd-tqmx86-refactor-I2C-setup.patch \
    file://0016-mfd-tqmx86-add-detection-for-MachXO2-I2C-controller.patch \
    file://0017-mfd-tqmx86-add-detection-for-secondary-I2C-controlle.patch \
    file://0018-gpio-tqmx86-introduce-platform-data-struct.patch \
    file://0019-mfd-tqmx86-update-TQMX86_IOSIZE_GPIO.patch \
    file://0020-mfd-tqmx86-configure-board-specific-GPIO-platform-da.patch \
    file://0021-gpio-tqmx86-prepare-support-for-variants-with-more-G.patch \
    file://0022-gpio-tqmx86-add-support-for-up-to-16-GPIOs.patch \
    file://0023-mfd-tqmx86-add-board-definition-for-TQMxE41M.patch \
    \
    file://nct7802.cfg \
    file://tqmx86.cfg \
"

KMACHINE:intel-x86-64-tqmx86 = "intel-x86-64"

# Use latest kern-tools for compatibility with yocto-kernel-cache 6.12
python () {
    import re

    for task in [
        'do_kernel_metadata',
        'do_validate_branches',
        'do_kernel_configme',
        'do_config_analysis',
    ]:
        depends = d.getVarFlag(task, 'depends')
        depends = re.sub(r'\bkern-tools-native\b', r'kern-tools-tqmx86-native', depends)
        d.setVarFlag(task, 'depends', depends)
}
