# find required GTK dependency
find_package(GTK)

include_directories (${GTK_INCLUDE_DIR})

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

target_link_libraries(ui.gtk ${GTK_LIBRARIES})
