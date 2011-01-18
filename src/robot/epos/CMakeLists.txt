add_library(epos
	epos.cc
	epos_access_rs232.cc epos_access_usb.cc)
	
add_executable(test test.cc)
target_link_libraries (test epos ftdi) 