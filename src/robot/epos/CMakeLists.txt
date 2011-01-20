if(LibFTDI_FOUND)
    add_library(epos
    	epos.cc
    	epos_access_rs232.cc epos_access_usb.cc)
    	
    target_link_libraries(epos ${LibFTDI_LIBRARIES})
else(LibFTDI_FOUND)
    add_library(epos
        	epos.cc
        	epos_access_rs232.cc)    	
endif(LibFTDI_FOUND)	

# Test utility
#add_executable(test test.cc)
#target_link_libraries (test epos) 
