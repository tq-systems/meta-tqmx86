FILESEXTRAPATHS_prepend_tqmx86 := "${THISDIR}/common:${THISDIR}/${PN}:"

# Switch to linux-stable repo to get stable releases not available in Intel's repo
SRC_URI = " \
	git://git.kernel.org/pub/scm/linux/kernel/git/stable/linux.git;protocol=https;name=machine;branch=${KBRANCH} \
	git://git.yoctoproject.org/yocto-kernel-cache;type=kmeta;name=meta;branch=${KMETA_BRANCH};destsuffix=${KMETA} \
	file://0001-menuconfig-mconf-cfg-Allow-specification-of-ncurses-.patch \
"

LINUX_VERSION = "5.15.25"
KBRANCH = "linux-5.15.y"
KMETA_BRANCH = "yocto-5.15"
SRCREV_machine = "1e7beca2829960d7ec407d0a7b3f5243c6344412"
SRCREV_meta = "c4e4de6ccb27846e48a848d7ca1f20d9503a6fec"

SRC_URI_append_tqmx86 = "\
	file://0001-mfd-tqmx86-fix-IO-port-base-register.patch \
	file://0002-gpio-tqmx86-store-output-register-value-in-driver-da.patch \
	file://0003-gpio-tqmx86-add-support-for-changing-GPIO-directions.patch \
	file://0004-gpio-tqmx86-prepare-support-for-variants-with-more-G.patch \
	file://0005-gpio-tqmx86-add-support-for-registers-of-variants-wi.patch \
	file://0006-mfd-tqmx86-correct-board-names-for-TQMxE39x.patch \
	file://0007-gpio-tqmx86-prepare-support-for-variants-with-more-G.patch \
	file://0008-mfd-tqmx86-use-platform_data-dependent-on-board_id.patch \
	file://0009-gpio-gpio-tqmx86-fix-broken-IRQ_TYPE_EDGE_BOTH-inter.patch \
	file://0010-gpio-gpio-tqmx86-add-support-for-new-interrupt-regis.patch \
	file://0011-mfd-tqmx86-Add-platform_data-for-smarc-with-14-gpios.patch \
	file://0012-Revert-can-m_can-pci-use-custom-bit-timings-for-Elkh.patch \
	file://0013-can-m_can-remove-support-for-custom-bit-timing-take-.patch \
	file://0014-i2c-busses-i2c-ocores-generate-stop-condition-after-.patch \
	file://0015-mfd-tqmx86-add-irq-option-for-ocores-i2c.patch \
	\
	file://elkhart-lake.cfg \
	file://tqmx86.cfg \
	file://nct7802.cfg \
"

KERNEL_EXTRA_FEATURES_append = " features/can/m_can.scc cfg/debug/mem/debug-strict-devmem.cfg"
