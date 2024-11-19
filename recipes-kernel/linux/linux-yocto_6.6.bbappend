require linux-yocto-tqmx86-6.6.inc

# Fixed version/SRCREV to make sure that our patches remain applicable
LINUX_VERSION:tqmx86 ?= "6.6.62"
SRCREV_machine:tqmx86 ?= "c0b4a8f13b976065abdeaa2f1177dbc5cb600b4b"
SRCREV_meta:tqmx86 ?= "8cda7c0eb6978af45b1f41e17f325056536c1d53"
