/////////////////////////////////////////////////////////
// Name: CubeMoveConst.h
// Implements: CubeMoveConst
/////////////////////////////////////////////////////////

#include "vsp/rcs/Cube/CubeMove.h"


const char CubeMove::MOVE_NAMES[CubeMove::MOVE_NUMBER+1] = "FRUBLD";
const char CubeMove::TURN_NAMES[TURN_NUMBER+1] = "123";

CubeMove::eMove CubeMove::MOVE_NUMBER_FROM_NAME(char name)
    throw (CubeException)
{
    const char *pch = strchr(CubeMove::MOVE_NAMES, name);
    if (pch == NULL)
        throw CubeException(NULL, NULL, name, 
            CubeException::ERR_CMOVE_INVALID_MOVE_NAME);
    return (CubeMove::eMove) (pch-CubeMove::MOVE_NAMES);
}

CubeMove::eTurn CubeMove::TURN_NUMBER_FROM_NAME(char name)
    throw (CubeException)
{
    const char *pch = strchr(CubeMove::TURN_NAMES, name);
    if (pch == NULL)
        throw CubeException(NULL, NULL, name, 
            CubeException::ERR_CMOVE_INVALID_TURN_NAME);
    return (CubeMove::eTurn) (pch-CubeMove::TURN_NAMES);
}

CubeMove::eMove CubeMove::OPPOSITE_FACE_MOVE(CubeMove::eMove move)
{
    return (eMove) ((move + 3) % MOVE_NUMBER);
}

CubeMove::eTurn CubeMove::REVERSE_TURN(CubeMove::eTurn turn)
{
    return (eTurn) (TURN_NUMBER - turn - 1);
}

