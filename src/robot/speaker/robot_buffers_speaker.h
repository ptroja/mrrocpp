/*!
 * @file robot_buffers_speaker.h
 * @brief File contains constants and structures for IRp6 on track manipulator
 * @author Piotr Trojanek <piotr.trojanek@gmail.com>, Warsaw University of Technology
 *
 * @ingroup speaker
 */

#ifndef ROBOT_BUFFERS_SPEAKER_H_
#define ROBOT_BUFFERS_SPEAKER_H_

#include "base/lib/com_buf.h"

namespace mrrocpp {
namespace lib {
namespace speaker {

struct c_buffer
{
#ifndef USE_MESSIP_SRR
	/*! This is a message buffer, so it needs a message header */
	msg_header_t hdr;
#endif

	/*! Type of the instruction. */
	INSTRUCTION_TYPE instruction_type;

	struct
	{
		/*! Text to speak */
		char text[lib::MAX_TEXT];
		/*! Prosody of the text to speak */
		char prosody[lib::MAX_PROSODY];
	} text_def;

	//! Give access to boost::serialization framework
	friend class boost::serialization::access;

	//! Serialization of the data structure
	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		ar & instruction_type;
		ar & text_def.text;
		ar & text_def.prosody;
	}

	c_buffer(void) :
		  instruction_type(SYNCHRO)
	{}
};

struct r_buffer : lib::r_buffer_base
{
	/*!
	 *  Czy mowi?
	 *  @todo Translate to English.
	 */
	bool speaking;

	//! Give access to boost::serialization framework
	friend class boost::serialization::access;

	//! Serialization of the data structure
	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		// serialize base class information
		ar & boost::serialization::base_object<lib::r_buffer_base>(*this);
		ar & speaking;
	}
};

} // namespace speaker
} // namespace lib
} // namespace mrrocpp

#endif /* ROBOT_BUFFERS_SPEAKER_ */
