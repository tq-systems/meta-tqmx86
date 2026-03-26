SUMMARY = "Program EDID information in NXP PTN3460 eDP to LVDS bridges"

SRC_URI = "git://${TQMX86_GIT_ROOT}/ptn3460-edid-tool.git;protocol=${TQMX86_GIT_PROTOCOL};branch=main"
SRCREV = "98a6c8feb679648c2aa845b0a5b133f63e01ce10"

LICENSE = "GPL-2.0-or-later"
LIC_FILES_CHKSUM = "file://COPYING;md5=751419260aa954499f7abaabaa882bbe"

inherit cmake

do_install() {
    install -d ${D}${bindir}
    install -m0755 ${B}/ptn3460-edid-tool ${D}${bindir}/
}
