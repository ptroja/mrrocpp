# find required GTK dependency
find_package(PkgConfig)

pkg_check_modules(GTKMM REQUIRED gtkmm-2.4)
pkg_check_modules(GTK2 gtk+-2.0)
pkg_check_modules(GTHREAD gthread-2.0)

if(GTKMM_FOUND AND GTK2_FOUND AND GTHREAD_FOUND)
include_directories(${GTKMM_INCLUDE_DIRS})

# GLADE definitions for UI modules
set(ui_defs mp.xml)

# Libraries for UI modules
set(ui_modules mp.so)

add_subdirectory(generator)
add_subdirectory(glade)
add_subdirectory(signals)

add_executable(ui.gtk
	main.cc
	xmlconfigurator.cc
	ui_model.cc
	ui_config_entry.cc
	ui_widget_entry.cc
	sr_console.cc
	comm_thread.cc
	../src/ui.cc
	../src/threads.cc
)

target_link_libraries(ui.gtk ${GTKMM_LIBRARIES} ${GTK2_LIBRARIES} ${GTHREAD_LIBRARIES} ${LIBXML2_LIBRARIES} ${COMMON_LIBRARIES})

install(TARGETS ui.gtk DESTINATION bin)

# TODO: add ${CMAKE_CURRENT_SOURCE_DIR}/start.sh as a dependency
ADD_CUSTOM_TARGET(start.sh ALL
	COMMAND cmake -E copy ${CMAKE_CURRENT_SOURCE_DIR}/start.sh ${CMAKE_CURRENT_BINARY_DIR}/start.sh
)

#install(TARGETS start.sh DESTINATION bin)

endif(GTKMM_FOUND AND GTK2_FOUND AND GTHREAD_FOUND)
