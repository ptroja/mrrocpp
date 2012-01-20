# Recurse into subdirectories.
if(NOT UBUNTU32BIT)
add_subdirectory (ecp)
add_subdirectory (mp)
endif(NOT UBUNTU32BIT)