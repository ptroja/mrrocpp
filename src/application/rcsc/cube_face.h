/*!
 * \file cube_face.h
 * \brief Contains declaration of structure representing single (identified) cube face, sent by FraDIA.
 * 
 * \author  tkornuta
 * \date Aug 4, 2010
 */


#ifndef CUBE_FACE_H_
#define CUBE_FACE_H_

namespace mrrocpp {
namespace mp {
namespace task {

/**
 * @brief Structure used in the cube state recognition task.
 */
typedef struct _cube_face
{
	char colors[9];
} cube_face_t;

} // namespace task
} // namespace mp
} // namespace mrrocpp


#endif /* CUBE_FACE_H_ */
