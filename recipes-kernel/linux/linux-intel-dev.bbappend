FILESEXTRAPATHS_prepend_tqmx86 := "${THISDIR}/files:"

# Switch to linux-stable repo to get stable releases not available in Intel's repo
SRC_URI = " \
	git://git.kernel.org/pub/scm/linux/kernel/git/stable/linux.git;protocol=https;name=machine;branch=${KBRANCH} \
	git://git.yoctoproject.org/yocto-kernel-cache;type=kmeta;name=meta;branch=${KMETA_BRANCH};destsuffix=${KMETA} \
	file://0001-menuconfig-mconf-cfg-Allow-specification-of-ncurses-.patch \
"

LINUX_VERSION = "5.15.2"
KBRANCH = "linux-5.15.y"
KMETA_BRANCH = "yocto-5.15"
SRCREV_machine = "7cc36c3e14ae0af800a3a5d20cb17d0c168fc956"
SRCREV_meta = "5bcd50d6b0cde22583768dc1acf304259364ddbc"

SRC_URI_append_tqmx86 = " \
	file://m_can-0001-can-m_can-pci-fix-incorrect-reference-clock-rate.patch \
	file://m_can-0002-Revert-can-m_can-remove-support-for-custom-bit-timin.patch \
	file://m_can-0003-can-m_can-make-custom-bittiming-fields-const.patch \
	file://m_can-0004-can-m_can-pci-use-custom-bit-timings-for-Elkhart-Lak.patch \
	file://m_can-0005-can-m_can-pci-fix-iomap_read_fifo-and-iomap_write_fi.patch \
	\
	file://elkhart-lake.cfg \
	file://tqmx86.cfg \
	file://nct7802.cfg \
"

KERNEL_EXTRA_FEATURES_append = " features/can/m_can.scc cfg/debug/mem/debug-strict-devmem.cfg"
