
if(COMEDILIB_FOUND)

add_library(ati3084KB
        edp_s.cc
)

target_link_libraries(
        ati3084KB
        ${COMEDI_LIBRARY}
)
install(TARGETS ati3084KB DESTINATION lib)
endif(COMEDILIB_FOUND)
