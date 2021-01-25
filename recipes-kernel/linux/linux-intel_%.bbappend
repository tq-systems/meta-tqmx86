FILESEXTRAPATHS_prepend_tqmx86 := "${THISDIR}/files:"

SRC_URI_append_tqmx86 = " \
	file://0001-gpio-tqmx86-really-make-IRQ-optional.patch \
	file://0002-i2c-ocores-do-not-print-error-without-IRQ.patch \
	file://0003-mfd-tqmx86-clear-GPIO-IRQ-resource-when-no-IRQ-is-se.patch \
	file://0004-mfd-tqmx86-add-support-for-TQMxE40M.patch \
	\
	file://elkhart-lake.cfg \
	file://tqmx86.cfg \
	file://nct7802.cfg \
"
