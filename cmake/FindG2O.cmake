# This file is copied and modified from g2o official repository


# Find the header files
FIND_PATH(G2O_INCLUDE_DIR g2o/core/base_vertex.h
  ${G2O_ROOT}/include
  $ENV{G2O_ROOT}/include
  $ENV{G2O_ROOT}
  /usr/local/include
  /usr/include
  /opt/local/include
  /sw/local/include
  /sw/include
  NO_DEFAULT_PATH
  )

# Macro to unify finding both the debug and release versions of the
# libraries; this is adapted from the OpenSceneGraph FIND_LIBRARY
# macro.
MACRO(FIND_G2O_LIBRARY MYLIBRARY MYLIBRARYNAME)

  FIND_LIBRARY("${MYLIBRARY}_DEBUG"
    NAMES "g2o_${MYLIBRARYNAME}_d"
    PATHS
    ${G2O_ROOT}/lib/Debug
    ${G2O_ROOT}/lib
    $ENV{G2O_ROOT}/lib/Debug
    $ENV{G2O_ROOT}/lib
    NO_DEFAULT_PATH
    )

  FIND_LIBRARY("${MYLIBRARY}_DEBUG"
    NAMES "g2o_${MYLIBRARYNAME}_d"
    PATHS
    ~/Library/Frameworks
    /Library/Frameworks
    /usr/local/lib
    /usr/local/lib64
    /usr/lib
    /usr/lib64
    /opt/local/lib
    /sw/local/lib
    /sw/lib
    )

  FIND_LIBRARY(${MYLIBRARY}
    NAMES "g2o_${MYLIBRARYNAME}"
    PATHS
    ${G2O_ROOT}/lib/Release
    ${G2O_ROOT}/lib
    $ENV{G2O_ROOT}/lib/Release
    $ENV{G2O_ROOT}/lib
    NO_DEFAULT_PATH
    )

  FIND_LIBRARY(${MYLIBRARY}
    NAMES "g2o_${MYLIBRARYNAME}"
    PATHS
    ~/Library/Frameworks
    /Library/Frameworks
    /usr/local/lib
    /usr/local/lib64
    /usr/lib
    /usr/lib64
    /opt/local/lib
    /sw/local/lib
    /sw/lib
    )

  IF(NOT ${MYLIBRARY}_DEBUG)
    IF(MYLIBRARY)
      SET(${MYLIBRARY}_DEBUG ${MYLIBRARY})
    ENDIF(MYLIBRARY)
  ENDIF( NOT ${MYLIBRARY}_DEBUG)

ENDMACRO(FIND_G2O_LIBRARY LIBRARY LIBRARYNAME)

# Find the core elements
FIND_G2O_LIBRARY(G2O_CORE_LIBRARY core)

# Find the predefined types
FIND_G2O_LIBRARY(G2O_TYPES_SLAM3D types_slam3d)

# G2O itself declared found if we found the core libraries and at least one solver
SET(G2O_FOUND "NO")
IF(G2O_CORE_LIBRARY AND G2O_INCLUDE_DIR)
  SET(G2O_FOUND "YES")
ENDIF(G2O_CORE_LIBRARY AND G2O_INCLUDE_DIR)
