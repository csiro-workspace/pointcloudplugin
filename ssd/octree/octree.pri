#
# vim:filetype=qmake
#

OCTREEDIR = src/octree

HEADERS += \
    $${OCTREEDIR}/OctreeBase.hpp \
    $${OCTREEDIR}/HashOctree.hpp \
    $${OCTREEDIR}/HashOctree.inl \
    $${OCTREEDIR}/OctreeDataBase.hpp \
    $${OCTREEDIR}/OctreePointData.hpp \
    $${OCTREEDIR}/OctreePointData.inl \
    $${OCTREEDIR}/OctreeBundle.hpp \
    $${OCTREEDIR}/OctreeBundle.inl \
    $${OCTREEDIR}/SimpleHash.hpp \
    $${OCTREEDIR}/Morton.hpp \
    $${OCTREEDIR}/SSDData.hpp \
    $${OCTREEDIR}/SmoothSignedDistance.hpp \
    $${OCTREEDIR}/SmoothSignedDistance.inl \
    $${OCTREEDIR}/ColorSmoothSignedDistance.hpp \
    $${OCTREEDIR}/ColorSmoothSignedDistance.inl \
    $${OCTREEDIR}/OctreeCache.hpp \
    $${OCTREEDIR}/OctreeCache.inl \
    $$(NULL)

SOURCES += \
    $${OCTREEDIR}/Morton.cpp \
    $$(NULL)
