PROJECT(ChopperRemote)
SET(CMAKE_MODULE_PATH ${ChopperRemote_SOURCE_DIR})
SET(ChopperRemote_SOVERSION 1)

FIND_PACKAGE(Boost 1.35 REQUIRED)
FIND_PACKAGE(Qt4 REQUIRED)

ADD_DEFINITIONS(-DWT_THREADED -D_REENTRANT -DBOOST_SPIRIT_THREADSAFE)

# Make sure the compiler can find include files.
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR} ${ChopperRemote_SOURCE_DIR} ${ChopperRemote_CLIENT_BINARY_DIR})
INCLUDE_DIRECTORIES(/usr/include/qwt-qt4 ${PROJECT_SOURCE_DIR} ${BOOST_INCLUDE_DIRS} ${CMAKE_CURRENT_BINARY_DIR})
INCLUDE(${QT_USE_FILE})

LINK_DIRECTORIES(${BOOST_LIB_DIRS})

SET(ChopperRemote_SRCS main.cpp mainwindow.cpp ProximityRadar.cpp Communication.cpp Attitude.cpp joystick.cpp)
SET(ChopperRemote_MOC_HDRS mainwindow.h ProximityRadar.h Attitude.h)
QT4_WRAP_CPP(ChopperRemote_SRCS ${ChopperRemote_MOC_HDRS})

ADD_EXECUTABLE(ChopperRemote ${ChopperRemote_SRCS})

TARGET_LINK_LIBRARIES(ChopperRemote boost_system-mt boost_thread-mt ${QT_LIBRARIES} qwt-qt4)

SET_TARGET_PROPERTIES(
    ChopperRemote
  PROPERTIES
    VERSION ${VERSION_SERIES}.${VERSION_MAJOR}.${VERSION_MINOR}
    SOVERSION ${ChopperRemote_SOVERSION}
    DEBUG_POSTFIX "d"
  )

INSTALL(TARGETS ChopperRemote RUNTIME DESTINATION bin)

