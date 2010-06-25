include_directories (
	${CMAKE_CURRENT_SOURCE_DIR}/ati
	${CMAKE_CURRENT_SOURCE_DIR}/ni
	${CMAKE_CURRENT_SOURCE_DIR}
)

if(QNXNTO)
set(QNX_ATI6284_SOURCES
	edp_s.cc
	osiUserCode.cc
)
endif(QNXNTO)

set(PORTABLE_ATI6284_SOURCES
	dom.cc
	expatls.cc
	ftconfig.cc
	ftconvert.cc
	ftrt.cc
	node.cc
	osiBus.cc
	stack.cc
	tESeries.cc
	tSTC.cc
	xmlparse.cc
	xmlrole.cc
	xmltok.cc
)

add_library(ati6284
	${QNX_ATI6284_SOURCES}
	${PORTABLE_ATI6284_SOURCES}
)
