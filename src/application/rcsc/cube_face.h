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

    //! Give access to boost::serialization framework
    friend class boost::serialization::access;

    //! Serialization of the data structure
    template <class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & colors;
    }
} cube_face_t;

} // namespace task
} // namespace mp
} // namespace mrrocpp


#endif /* CUBE_FACE_H_ */
