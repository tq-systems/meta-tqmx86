FILESEXTRAPATHS_prepend_tqmx86 := "${THISDIR}/files:"

# Switch to linux-stable repo to get stable releases not available in Intel's repo
SRC_URI = " \
	git://git.kernel.org/pub/scm/linux/kernel/git/stable/linux.git;protocol=https;name=machine;branch=${KBRANCH} \
	git://git.yoctoproject.org/yocto-kernel-cache;type=kmeta;name=meta;branch=${KMETA_BRANCH};destsuffix=${KMETA} \
	file://0001-menuconfig-mconf-cfg-Allow-specification-of-ncurses-.patch \
"

LINUX_VERSION = "5.14.9"
KBRANCH = "linux-5.14.y"
KMETA_BRANCH = "yocto-5.14"
SRCREV_machine = "70248e7b378b96f208d5544ee25b808a8ef2ddc2"
SRCREV_meta = "884dfea956ec6b166d1f99a295c47338573a974c"

SRC_URI_append_tqmx86 = " \
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

KERNEL_EXTRA_FEATURES_append = " features/can/m_can.scc cfg/debug/mem/debug-strict-devmem.cfg"
