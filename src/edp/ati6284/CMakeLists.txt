include_directories (
	${CMAKE_CURRENT_SOURCE_DIR}/ati
	${CMAKE_CURRENT_SOURCE_DIR}/ni
	${CMAKE_CURRENT_SOURCE_DIR}
)

add_library(ati6284
	dom.cc
	edp_s.cc
	expatls.cc
	ftconfig.cc
	ftconvert.cc
	ftrt.cc
	node.cc
	osiBus.cc
	osiUserCode.cc
	stack.cc
	tESeries.cc
	tSTC.cc
	xmlparse.cc
	xmlrole.cc
	xmltok.cc
)
