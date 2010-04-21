# Recurse into subdirectories.
add_subdirectory (src)
if(NOT QNXNTO)
add_subdirectory (gtk)
endif(NOT QNXNTO)
