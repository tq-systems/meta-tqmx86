FILESEXTRAPATHS_prepend_tqmx86 := "${THISDIR}/files:"

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

SRC_URI_append_tqmx86 = " \
	file://elkhart-lake.cfg \
	file://tqmx86.cfg \
	file://nct7802.cfg \
"

KERNEL_EXTRA_FEATURES_append = " features/can/m_can.scc cfg/debug/mem/debug-strict-devmem.cfg"
