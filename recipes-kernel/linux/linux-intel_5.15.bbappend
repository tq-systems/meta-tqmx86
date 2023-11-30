FILESEXTRAPATHS:prepend:tqmx86 := "${THISDIR}/${PN}:"

LINUX_VERSION = "5.15.129"
SRCREV_machine = "6a3035823c5db6ee250a049f98a6d7bd53f837c7"
SRCREV_meta = "0b002d94afb8a3b60ed1f3be419cb9f5a8815cfc"

SRC_URI:append:tqmx86 = "\
	file://0001-gpio-gpio-tqmx86-fix-typo-in-Kconfig.patch \
	file://0002-gpio-tqmx86-store-output-register-value-in-driver-da.patch \
	file://0003-gpio-tqmx86-add-support-for-changing-GPIO-directions.patch \
	file://0004-gpio-tqmx86-prepare-support-for-variants-with-more-G.patch \
	file://0005-gpio-tqmx86-add-support-for-registers-of-variants-wi.patch \
	file://0006-gpio-tqmx86-prepare-support-for-variants-with-more-G.patch \
	file://0007-mfd-tqmx86-use-platform_data-dependent-on-board_id.patch \
	file://0008-gpio-gpio-tqmx86-fix-broken-IRQ_TYPE_EDGE_BOTH-inter.patch \
	file://0009-gpio-gpio-tqmx86-add-support-for-new-interrupt-regis.patch \
	file://0010-mfd-tqmx86-Add-platform_data-for-smarc-with-14-gpios.patch \
	file://0011-mfd-tqmx86-add-irq-option-for-ocores-i2c.patch \
	file://0012-mfd-tqmx86-change-board-ID-defines-to-hexadecimal.patch \
	file://0013-mfd-tqmx86-improve-IRQ-error-messages.patch \
	file://0014-mfd-tqmx86-make-IRQ-config-errors-non-fatal.patch \
	file://0015-mfd-tqmx86-make-GPIO-support-errors-non-fatal.patch \
	file://0016-mfd-tqmx86-add-board-definitions-for-TQMx120UC-and-T.patch \
	file://0017-mfd-tqmx86-add-board-definitions-for-TQMxE41S.patch \
	\
	file://elkhart-lake.cfg \
	file://nct7802.cfg \
	file://serial.cfg \
	file://tqmx86.cfg \
"

KERNEL_EXTRA_FEATURES:append:tqmx86 = " features/can/m_can.scc"
