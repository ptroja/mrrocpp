add_library(cpn c_cpn.c)

add_executable(receiver receiver.c)
target_link_libraries(receiver cpn)

if(QNXNTO)
target_link_libraries(receiver socket)
endif(QNXNTO)
