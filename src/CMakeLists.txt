# TODO: remove these utility macros to separate file
MACRO(TARGET_LINK_LIBRARY_IF CONDITION TARGET LIBRARY)
	set(CONDITION_VALUE ${${CONDITION}})
	if(CONDITION_VALUE)
		target_link_libraries(${TARGET} ${LIBRARY})
	endif(CONDITION_VALUE)
ENDMACRO(TARGET_LINK_LIBRARY_IF)

## this is currently disabled
## as CMake option names duplicates _ROBOT_ENUM values
# define both CMake option and -D flag for compiler
#MACRO(OPTION_AND_DEFINITION OPTION DESCRIPTION DEFAULT_VALUE)
#	option(${OPTION} ${DESCRIPTION} ${DEFAULT_VALUE})
#	set(OPTION_VALUE ${${OPTION}})
#	if(OPTION_VALUE)
#		add_definitions("-D${OPTION}")
#	endif(OPTION_VALUE)
#ENDMACRO(OPTION_AND_DEFINITION)

option(ROBOT_CONVEYOR "Build support for conveyor robot" ON)
option(ROBOT_IRP6_ON_TRACK "Build support for IRp6 'on track' robot" ON)
CMAKE_DEPENDENT_OPTION(ROBOT_IRP6_ON_TRACK_M
	"Build support for IRp6 'on track' robot without a gripper"
	ON "ROBOT_IRP6_ON_TRACK" OFF)
CMAKE_DEPENDENT_OPTION(ROBOT_IRP6_ON_TRACK_TFG
	"Build support for gripper mounted on IRp6 'on track' robot"
	ON "ROBOT_IRP6_ON_TRACK" OFF)
option(ROBOT_IRP6_POSTUMENT "Build support for IRp6 'postument' robot" ON)
CMAKE_DEPENDENT_OPTION(ROBOT_IRP6_POSTUMENT_M
	"Build support for IRp6 'postument' robot without a gripper"
	ON "ROBOT_IRP6_POSTUMENT" OFF)
CMAKE_DEPENDENT_OPTION(ROBOT_IRP6_POSTUMENT_TFG
	"Build support for gripper mounted on IRp6 'postument' robot"
	ON "ROBOT_IRP6_POSTUMENT" OFF)
option(ROBOT_IRP6_MECHATRONICS "Build support for IRp6 robot located at Mechatronics Deptartment" ON)
option(ROBOT_POLYCRANK "Build support for Polycrank robot" ON)
option(ROBOT_SPEAKER "Build support for speaker robot" ON)

option(ROBOTS_SWARMITFIX "Build support for SwarmItFIX robots" ON)
CMAKE_DEPENDENT_OPTION(ROBOT_SHEAD
	"Build support for SwarmItFIX 'head' robot"
	ON "ROBOTS_SWARMITFIX" OFF)
CMAKE_DEPENDENT_OPTION(ROBOT_SMB
	"Build support for SwarmItFIX mobile base robot"
	ON "ROBOTS_SWARMITFIX" OFF)
CMAKE_DEPENDENT_OPTION(ROBOT_SPKM
	"Build support for SwarmItFIX parallel kinematics machine robot"
	ON "ROBOTS_SWARMITFIX" OFF)

# Recurse into subdirectories.
add_subdirectory (lib)
add_subdirectory (kinematics)
add_subdirectory (edp)
add_subdirectory (ecp)
add_subdirectory (ecp_mp)
add_subdirectory (mp)
add_subdirectory (ui)
add_subdirectory (application)
add_subdirectory (vsp)
