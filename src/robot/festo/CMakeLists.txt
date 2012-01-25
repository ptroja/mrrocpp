add_library(festo cpv.cc)
	
target_link_libraries(festo canopen)

install(TARGETS festo DESTINATION lib)
