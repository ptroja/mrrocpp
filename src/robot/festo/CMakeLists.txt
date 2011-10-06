add_library(festo cpv.cc)
	
target_link_libraries(festo canopen)

# Test utilities
add_executable(cpv_test cpv_test.cc)
target_link_libraries (cpv_test epos festo rt)

install(TARGETS festo DESTINATION lib)
