# Try to find and setup Qt4 for this project
FIND_PACKAGE(Qt4)

IF(QT_FOUND)

MESSAGE(STATUS "\tUsing Qt ${QT_VERSION_MAJOR}.${QT_VERSION_MINOR}.${QT_VERSION_PATCH} (${QT_QMAKE_EXECUTABLE})")
MESSAGE(STATUS "\tIncludes in ${QT_INCLUDE_DIR}")

# add Qt modules here, the include will setup QT_LIBRARIES
#
# SET(QT_USE_XXX TRUE)
#
# Possible values are:
# QT_DONT_USE_QTCORE
# QT_DONT_USE_QTGUI
# QT_USE_QT3SUPPORT
# QT_USE_QTASSISTANT
# QT_USE_QAXCONTAINER
# QT_USE_QAXSERVER
# QT_USE_QTDESIGNER
# QT_USE_QTMOTIF
# QT_USE_QTMAIN
# QT_USE_QTNETWORK
# QT_USE_QTNSPLUGIN
# QT_USE_QTOPENGL
# QT_USE_QTSQL
# QT_USE_QTXML
# QT_USE_QTSVG
# QT_USE_QTTEST
# QT_USE_QTUITOOLS
# QT_USE_QTDBUS
# QT_USE_QTSCRIPT
# QT_USE_QTASSISTANTCLIENT
# QT_USE_QTHELP
# QT_USE_QTWEBKIT
# QT_USE_QTXMLPATTERNS
# QT_USE_PHONON

INCLUDE(${QT_USE_FILE})

# Include current directory, necessary for proper UI headers to work
include_directories(${CMAKE_CURRENT_BINARY_DIR})

# Source files
SET(GUI_CXX_SRCS
    main.cpp
    mainwindow.cpp
    spkm/wnd_spkm_inc.cpp
)

# Class headers that have to be MOCed (i.e. classes derived from QObject)
SET(GUI_MOC_SRCS
    mainwindow.h
    spkm/wnd_spkm_inc.h
)

# Generate MOC files
QT4_WRAP_CPP(GUI_CXX_SRCS ${GUI_MOC_SRCS})

# UI files
SET(GUI_UI
    mainwindow.ui
    spkm/wnd_spkm_inc.ui
)

# Generate UI include files
QT4_WRAP_UI(GUI_CXX_SRCS ${GUI_UI})

# Create GUI executable
ADD_EXECUTABLE(
	ui-qt-yoyek 
	${GUI_CXX_SRCS}
	ui_sr.cc
	interface.cc
	ui_ecp_r_data_port.cc
	spkm/ui_ecp_r_spkm.cc
	spkm/ui_r_spkm.cc
	smb/ui_ecp_r_smb.cc
	smb/ui_r_smb.cc
	shead/ui_ecp_r_shead.cc
	shead/ui_r_shead.cc
	ui.cc
	ui_robot.cc
)

# Link executable with QT libraries
TARGET_LINK_LIBRARIES(
	ui-qt-yoyek 
	ecp_robots ecp_mp
	${QT_LIBRARIES}
	${COMMON_LIBRARIES}
)

install(
  TARGETS ui-qt-yoyek
  RUNTIME DESTINATION bin COMPONENT applications
)

install(PROGRAMS start.sh DESTINATION bin/ )

ELSE(QT_FOUND)

    MESSAGE("Qt not found! GUI will not be built")

ENDIF(QT_FOUND)
