#
# Try to find the FreeImage library and include path.
# Once done this will define
#
# FREEIMAGE_FOUND
# FREEIMAGE_INCLUDE_PATH
# FREEIMAGE_LIBRARY
# 

IF (WIN32)
  FIND_PATH( FREEIMAGE_INCLUDE_PATH FreeImage.h
    ${PROJECT_SOURCE_DIR}/extern/FreeImage
    DOC "The directory where FreeImage.h resides")
  FIND_LIBRARY( FREEIMAGE_LIBRARY
    NAMES FreeImage freeimage
    PATHS
    ${PROJECT_SOURCE_DIR}/FreeImage
    DOC "The FreeImage library")
ELSE (WIN32)
  FIND_PATH( BASE_FREEIMAGE_INCLUDE_PATH FreeImage.h
    /usr/include
    /usr/local/include
    /sw/include
    /opt/local/include
    DOC "The directory where FreeImage.h resides")
  FIND_PATH( FREEIMAGE_PLUS_INCLUDE_PATH FreeImagePlus.h
    /usr/include
    /usr/local/include
    /sw/include
    /opt/local/include
    DOC "The directory where FreeImagePlus.h resides")
  FIND_LIBRARY( BASE_FREEIMAGE_LIBRARY
    NAMES FreeImage freeimage
    PATHS
    /usr/lib64
    /usr/lib/x86_64-linux-gnu
    /usr/lib
    /usr/local/lib64
    /usr/local/lib
    /sw/lib
    /opt/local/lib
    DOC "The FreeImage library")
  FIND_LIBRARY( FREEIMAGE_PLUS_LIBRARY
    NAMES FreeImagePlus freeimageplus
    PATHS
    /usr/lib64
    /usr/lib/x86_64-linux-gnu
    /usr/lib
    /usr/local/lib64
    /usr/local/lib
    /sw/lib
    /opt/local/lib
    DOC "The FreeImagePlus library")
ENDIF (WIN32)

SET(FREEIMAGE_INCLUDE_PATH ${BASE_FREEIMAGE_INCLUDE_PATH} ${FREEIMAGE_PLUS_INCLUDE_PATH})
SET(FREEIMAGE_LIBRARY ${BASE_FREEIMAGE_LIBRARY} ${FREEIMAGE_PLUS_LIBRARY})
SET(FREEIMAGE_LIBRARIES ${FREEIMAGE_LIBRARY})

IF (FREEIMAGE_INCLUDE_PATH AND FREEIMAGE_LIBRARY)
  SET( FREEIMAGE_FOUND TRUE CACHE BOOL "Set to TRUE if FreeImage is found, FALSE otherwise")
ELSE (FREEIMAGE_INCLUDE_PATH AND FREEIMAGE_LIBRARY)
  SET( FREEIMAGE_FOUND FALSE CACHE BOOL "Set to TRUE if FreeImage is found, FALSE otherwise")
ENDIF (FREEIMAGE_INCLUDE_PATH AND FREEIMAGE_LIBRARY)

MARK_AS_ADVANCED(
  FREEIMAGE_FOUND 
  FREEIMAGE_LIBRARIES
  FREEIMAGE_INCLUDE_PATH)
