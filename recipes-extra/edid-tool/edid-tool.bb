SUMMARY = "Tool to write EDID data to PTN3460 IC"

SRC_URI = " \
    file://src/ \
"

LICENSE = "GPL-2.0-or-later"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/GPL-2.0-or-later;md5=fed54355545ffd980b814dab4a3b312c"

S = "${WORKDIR}/src"

do_install () {
	install -d ${D}${bindir}
	install -m 0755 edid-tool ${D}${bindir}
}
