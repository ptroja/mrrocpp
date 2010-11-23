
if(QNXNTO)

else(QNXNTO)
add_library(ati6284KB
        edp_s.cc
)

target_link_libraries(
        ati6284KB
        comedi
)

endif(QNXNTO)
