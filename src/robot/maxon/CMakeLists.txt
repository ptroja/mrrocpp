add_library(epos epos.cc)
	
target_link_libraries(epos canopen)

# Test utilities
add_executable(test test.cc)
target_link_libraries (test epos rt)

add_executable(sctest sctest.cc)
target_link_libraries (sctest epos rt)

add_executable(ipmtest ipmtest.cc)
target_link_libraries (ipmtest epos rt)

add_executable(cantest cantest.cc)
target_link_libraries (cantest epos)

install(TARGETS epos DESTINATION lib)
