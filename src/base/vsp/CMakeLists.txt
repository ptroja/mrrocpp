if(QNXNTO)
add_library(vsp_int
    vsp_sensor_interface.cc
    vsp_m_int.cc
)

add_library(vsp_nint
    vsp_sensor_interface.cc
    vsp_m_nint.cc
)

add_library(vsp_int_nw
    vsp_sensor_interface.cc
#    vsp_m_int_nw.cc
)
endif(QNXNTO)