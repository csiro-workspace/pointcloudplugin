#
# vim:filetype=qmake
#

DGPDIR = src/dgp

HEADERS += \
    $${DGPDIR}/BBox.h \
    $${DGPDIR}/inttypes.h \
    $${DGPDIR}/PointXYZ.hpp \
    $${DGPDIR}/PointXYZNormal.hpp \
    $${DGPDIR}/PointXYZRGB.hpp \
    $${DGPDIR}/PointXYZRGBNormal.hpp \
    $$(NULL)

SOURCES += \
    $${DGPDIR}/BBox.cpp \
    $$(NULL)
 
