FILESEXTRAPATHS:prepend:tqmx86 := "${THISDIR}/${PN}:"

LINUX_VERSION = "5.15.85"
SRCREV_machine = "c256f934aefa78ec001067313a76a4a382ac59a6"
SRCREV_meta = "78c4410c172946903e35ba8cebf1cf90fad09b5a"

SRC_URI:append:tqmx86 = "\
	file://0001-mfd-tqmx86-do-not-access-I2C_DETECT-register-through.patch \
	file://0002-mfd-tqmx86-specify-IO-port-register-range-more-preci.patch \
	file://0003-mfd-tqmx86-correct-board-names-for-TQMxE39x.patch \
	file://0004-i2c-ocores-generate-stop-condition-after-timeout.patch \
	file://0005-gpio-gpio-tqmx86-fix-typo-in-Kconfig.patch \
	file://0006-gpio-tqmx86-store-output-register-value-in-driver-da.patch \
	file://0007-gpio-tqmx86-add-support-for-changing-GPIO-directions.patch \
	file://0008-gpio-tqmx86-prepare-support-for-variants-with-more-G.patch \
	file://0009-gpio-tqmx86-add-support-for-registers-of-variants-wi.patch \
	file://0010-gpio-tqmx86-prepare-support-for-variants-with-more-G.patch \
	file://0011-mfd-tqmx86-use-platform_data-dependent-on-board_id.patch \
	file://0012-gpio-gpio-tqmx86-fix-broken-IRQ_TYPE_EDGE_BOTH-inter.patch \
	file://0013-gpio-gpio-tqmx86-add-support-for-new-interrupt-regis.patch \
	file://0014-mfd-tqmx86-Add-platform_data-for-smarc-with-14-gpios.patch \
	file://0015-mfd-tqmx86-add-irq-option-for-ocores-i2c.patch \
	file://0016-serial-8250-fix-duplicate-setup_irq-call-in-serial82.patch \
	\
	file://elkhart-lake.cfg \
	file://tqmx86.cfg \
	file://nct7802.cfg \
"

KERNEL_EXTRA_FEATURES:append:tqmx86 = " features/can/m_can.scc"
