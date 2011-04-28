if(LibFTDI_FOUND)
    include_directories (${LibFTDI_INCLUDE_DIRS})
    add_library(epos
    	epos.cc
    	epos_access_rs232.cc epos_access_usb.cc)
    	
    target_link_libraries(epos ${LibFTDI_LDFLAGS})

    # Test utility
    add_executable(test test.cc)
<<<<<<< HEAD
    target_link_libraries (test epos) 
=======
    target_link_libraries (test epos)
    
    add_executable(ipmtest ipmtest.cc)
    target_link_libraries (ipmtest epos) 

    add_executable(cantest cantest.cc)
    target_link_libraries (cantest epos) 
>>>>>>> 8a8d49a... ipmtest utility added
else(LibFTDI_FOUND)
    add_library(epos
        	epos.cc
        	epos_access_rs232.cc)    	
endif(LibFTDI_FOUND)	

install(TARGETS epos DESTINATION lib)
