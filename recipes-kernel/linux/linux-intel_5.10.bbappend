FILESEXTRAPATHS_prepend_tqmx86 := "${THISDIR}/files:"

LINUX_VERSION = "5.10.41"
SRCREV_machine = "916ac934a98bc8206c1b5e1500c891dbcb43aac7"
SRCREV_meta = "cd049697e9b2d3e9118110d476075ff8f87202cc"

SRC_URI_append_tqmx86 = " \
	file://0001-Revert-stmmac-align-RX-buffers.patch \
	\
	file://0001-i2c-ocores-Avoid-false-positive-error-log-message.patch \
	file://0002-gpio-tqmx86-really-make-IRQ-optional.patch \
	file://0003-mfd-tqmx86-clear-GPIO-IRQ-resource-when-no-IRQ-is-se.patch \
	file://0004-mfd-tqmx86-fix-typo-in-platform.patch \
	file://0005-mfd-tqmx86-remove-incorrect-TQMx90UC-board-ID.patch \
	file://0006-mfd-tqmx86-add-support-for-TQMx110EB-and-TQMxE40x.patch \
	file://0007-mfd-tqmx86-add-support-for-TQ-Systems-DMI-IDs.patch \
	file://0008-mfd-tqmx86-assume-24MHz-LPC-clock-for-unknown-boards.patch \
	\
	file://elkhart-lake.cfg \
	file://tqmx86.cfg \
	file://nct7802.cfg \
"
