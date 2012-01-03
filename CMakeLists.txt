CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

PROJECT(ChopperControl)
SET(CMAKE_MODULE_PATH ${ChopperControl_SOURCE_DIR})
SET(flightpred_MAIN_DIR ${ChopperControl_SOURCE_DIR})

SET(VERSION_SERIES 0)
SET(VERSION_MAJOR 0)
SET(VERSION_MINOR 2)

# Setup output Directories
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin CACHE PATH "Single Directory for all Libraries")
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin CACHE PATH "Single Directory for all Executables.")
SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin CACHE PATH "Single Directory for all static libraries.")


#SET(USERLIB_ROOT /usr CACHE PATH "Other installation prefix for dependent libraries")
#SET(LIB_INSTALL_DIR "lib" CACHE STRING "Default path for libraries within ${CMAKE_INSTALL_PREFIX}")
#SET(CONFIGDIR "/etc/flightpred" CACHE STRING "Path for the configuration files")
#SET(BOOST_DIR /usr CACHE PATH "Boost installation path prefix")
#SET(BOOST_COMPILER gcc CACHE STRING "Boost libraries compiler signature")
#SET(BOOST_VERSION "1_38" CACHE STRING "Boost libraries version signature")


IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release CACHE STRING
      "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
      FORCE)
ENDIF(NOT CMAKE_BUILD_TYPE)


SUBDIRS(src)

