# QNX is default MRROC++ target
if (QNXNTO)
  #SOURCES = vsp_sensor.cc vsp_m_int_srr.cc
else (QNXNTO)
  #SOURCES = vsp_sensor.cc vsp_m_int_nw.cc vsp_m_int.cc vsp_m_nint.cc
endif (QNXNTO)
