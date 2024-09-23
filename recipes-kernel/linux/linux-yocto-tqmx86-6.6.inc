FILESEXTRAPATHS:prepend:tqmx86 := "${THISDIR}/linux-yocto-tqmx86-6.6:"

SRC_URI:append:tqmx86 = " \
    file://0001-ACPI-utils-Introduce-acpi_dev_uid_match-for-matching.patch \
    file://0002-pinctrl-intel-use-acpi_dev_uid_match-for-matching-_U.patch \
    file://0003-net-introduce-napi_is_scheduled-helper.patch \
    file://0004-can-m_can-Move-hrtimer-init-to-m_can_class_register.patch \
    file://0005-can-m_can-Write-transmit-header-and-data-in-one-tran.patch \
    file://0006-can-m_can-Implement-receive-coalescing.patch \
    file://0007-can-m_can-Implement-transmit-coalescing.patch \
    file://0008-can-m_can-Add-rx-coalescing-ethtool-support.patch \
    file://0009-can-m_can-Add-tx-coalescing-ethtool-support.patch \
    file://0010-can-m_can-Use-u32-for-putidx.patch \
    file://0011-can-m_can-Cache-tx-putidx.patch \
    file://0012-can-m_can-Use-the-workqueue-as-queue.patch \
    file://0013-can-m_can-Introduce-a-tx_fifo_in_flight-counter.patch \
    file://0014-can-m_can-Use-tx_fifo_in_flight-for-netif_queue-cont.patch \
    file://0015-can-m_can-Implement-BQL.patch \
    file://0016-can-m_can-Implement-transmit-submission-coalescing.patch \
    file://0017-can-m_can-remove-redundant-check-for-pm_clock_suppor.patch \
    file://0018-can-m_can-allow-keeping-the-transceiver-running-in-s.patch \
    file://0019-can-m_can-don-t-enable-transceiver-when-probing.patch \
    file://0020-can-m_can-Constify-struct-m_can_ops.patch \
    file://0021-can-m_can-Release-irq-on-error-in-m_can_open.patch \
    file://0022-can-m_can-Reset-coalescing-during-suspend-resume.patch \
    file://0023-can-m_can-Remove-coalesing-disable-in-isr-during-sus.patch \
    file://0024-can-m_can-Remove-m_can_rx_peripheral-indirection.patch \
    file://0025-can-m_can-Do-not-cancel-timer-from-within-timer.patch \
    file://0026-can-m_can-disable_all_interrupts-not-clear-active_in.patch \
    file://0027-can-m_can-Reset-cached-active_interrupts-on-start.patch \
    file://0028-can-m_can-Limit-coalescing-to-peripheral-instances.patch \
    file://0029-can-m_can-m_can_chip_config-mask-timestamp-wraparoun.patch \
    file://0030-can-m_can-enable-NAPI-before-enabling-interrupts.patch \
    file://0031-can-m_can-m_can_close-stop-clocks-after-device-has-b.patch \
    file://0032-can-m_can-set-init-flag-earlier-in-probe.patch \
    file://0033-can-m_can-fix-missed-interrupts-with-m_can_pci.patch \
    file://0034-mfd-tqmx86-add-board-definitions-for-TQMx120UC-TQMx1.patch \
    file://0035-mfd-tqmx86-improve-gpio_irq-module-parameter-descrip.patch \
    file://0036-mfd-tqmx86-refactor-GPIO-IRQ-setup.patch \
    file://0037-mfd-tqmx86-make-IRQ-setup-errors-non-fatal.patch \
    file://0038-mfd-tqmx86-add-I2C-IRQ-support.patch \
    file://0039-gpio-tqmx86-add-macros-for-interrupt-configuration.patch \
    file://0040-gpio-tqmx86-consistently-refer-to-IRQs-by-hwirq-numb.patch \
    file://0041-gpio-tqmx86-introduce-tqmx86_gpio_clrsetbits-helper.patch \
    file://0042-gpio-tqmx86-add-support-for-changing-GPIO-directions.patch \
    file://0043-gpio-tqmx86-introduce-platform-data-struct.patch \
    file://0044-mfd-tqmx86-update-TQMX86_IOSIZE_GPIO.patch \
    file://0045-mfd-tqmx86-configure-board-specific-GPIO-platform-da.patch \
    file://0046-gpio-tqmx86-prepare-support-for-variants-with-more-G.patch \
    file://0047-gpio-tqmx86-add-support-for-up-to-16-GPIOs.patch \
    \
    file://nct7802.cfg \
    file://tqmx86.cfg \
"

KMACHINE:intel-x86-64-tqmx86 = "intel-x86-64"
