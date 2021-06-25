FILESEXTRAPATHS_prepend_tqmx86 := "${THISDIR}/files:"

LINUX_VERSION = "5.12.0"
SRCREV_machine = "9f4ad9e425a1d3b6a34617b8ea226d56a119a717"
SRCREV_meta = "99570241ac88d6c7e32b6fccd83afce53816b275"

SRC_URI_append_tqmx86 = " \
	file://0001-gpio-tqmx86-really-make-IRQ-optional.patch \
	file://0003-mfd-tqmx86-clear-GPIO-IRQ-resource-when-no-IRQ-is-se.patch \
	file://0004-mfd-tqmx86-add-support-for-TQMxE40M.patch \
	\
	file://elkhart-lake.cfg \
	file://tqmx86.cfg \
	file://nct7802.cfg \
"

KERNEL_EXTRA_FEATURES_append = " features/can/m_can.scc cfg/debug/mem/debug-strict-devmem.cfg"
