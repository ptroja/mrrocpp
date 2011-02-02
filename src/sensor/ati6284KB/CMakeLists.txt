
if(COMEDILIB_FOUND)

add_library(ati6284KB
        edp_s.cc
)

target_link_libraries(
        ati6284KB
        ${COMEDI_LIBRARY}
)

endif(COMEDILIB_FOUND)
