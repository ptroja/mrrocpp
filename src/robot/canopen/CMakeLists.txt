if(LibFTDI_FOUND)
    include_directories (${LibFTDI_INCLUDE_DIRS})
    add_library(canopen
		gateway.cc
    	gateway_epos_rs232.cc gateway_epos_usb.cc gateway_socketcan.cc)
    	
    target_link_libraries(canopen ${LibFTDI_LDFLAGS}
			${Boost_THREAD_LIBRARY} ${Boost_DATE_TIME_LIBRARY})
else(LibFTDI_FOUND)
    add_library(canopen
		gateway.cc
    	gateway_epos_usb.cc gateway_socketcan.cc)  	
endif(LibFTDI_FOUND)	

install(TARGETS canopen DESTINATION lib)
