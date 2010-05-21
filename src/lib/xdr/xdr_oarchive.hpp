#ifndef XDR_OARCHIVE_HPP
#define XDR_OARCHIVE_HPP

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/is_bitwise_serializable.hpp>
#include <boost/archive/detail/oserializer.hpp>
#include <boost/archive/archive_exception.hpp>
#include <boost/archive/detail/register_archive.hpp>
#include <boost/config.hpp>
#include <boost/utility/enable_if.hpp>
#include <boost/type_traits/is_enum.hpp>
#include <boost/type_traits/is_array.hpp>
#include <boost/mpl/bool.hpp>

#include <rpc/rpc.h>

#define THROW_SAVE_EXCEPTION \
	boost::serialization::throw_exception( \
			boost::archive::archive_exception( \
				boost::archive::archive_exception::stream_error))

#define SAVE_A_TYPE(T, P) \
    /** conversion for T */ \
    xdr_oarchive &save_a_type(T const &t,boost::mpl::true_) { \
        if(!P(&xdrs, (T *) &t)) THROW_SAVE_EXCEPTION; \
        return *this; \
    } \
    \
    /** conversion for T[] */ \
    template<int N> \
    xdr_oarchive &save_a_type(T const (&t)[N],boost::mpl::false_) { \
        if(!xdr_vector(&xdrs, (char *)t, N, sizeof(T), (xdrproc_t) P)) THROW_SAVE_EXCEPTION; \
        return *this; \
    }

template <std::size_t size = 4096>
class xdr_oarchive
{
private:

	char buffer[size];
	XDR xdrs;

public:
	//! conversion for bool, special since bool != bool_t
	xdr_oarchive &save_a_type(bool const &t, boost::mpl::true_)
	{
		if (!xdr_bool(&xdrs, (bool_t *) &t))
			THROW_SAVE_EXCEPTION;
		return *this;
	}

	//! conversion for bool[], special since bool != bool_t
	template <int N>
	xdr_oarchive &save_a_type(bool const(&t)[N], boost::mpl::false_)
	{
		if (!xdr_vector(&xdrs, (char *) t, N, sizeof(bool), (xdrproc_t) xdr_bool))
			THROW_SAVE_EXCEPTION;
		return *this;
	}

	//! conversion for an enum
	template <class T>
	typename boost::enable_if <boost::is_enum <T>, xdr_oarchive &>::type
	save_a_type(T const &t, boost::mpl::true_)
	{
		if (!xdr_enum(&xdrs, (enum_t *) &t))
			THROW_SAVE_EXCEPTION;
		return *this;
	}

	//! conversion for std::string
	xdr_oarchive &save_a_type(std::string const &t, boost::mpl::true_)
	{
		char * p = (char *) t.c_str();
		if (!xdr_wrapstring(&xdrs, &p))
			THROW_SAVE_EXCEPTION;
		return *this;
	}

	//! conversion for 'const char *' string
	xdr_oarchive &save_a_type(const char *t, boost::mpl::false_)
	{
		if (!xdr_wrapstring(&xdrs, (char **)&t))
			THROW_SAVE_EXCEPTION;
		return *this;
	}

	SAVE_A_TYPE(char, xdr_char)
	SAVE_A_TYPE(double, xdr_double)
	SAVE_A_TYPE(float, xdr_float)
	SAVE_A_TYPE(int, xdr_int)
	SAVE_A_TYPE(long, xdr_long)
	SAVE_A_TYPE(short, xdr_short)
	SAVE_A_TYPE(unsigned char, xdr_u_char)
	SAVE_A_TYPE(unsigned int, xdr_u_int)
	SAVE_A_TYPE(unsigned long, xdr_u_long)
	SAVE_A_TYPE(unsigned short, xdr_u_short)

	/**
	 * Saving Archive Concept::is_loading
	 */
	typedef boost::mpl::bool_ <false> is_loading;

	/**
	 * Saving Archive Concept::is_saving
	 */
	typedef boost::mpl::bool_ <true> is_saving;

	/**
	 * Constructor
	 * @param flags
	 */
	xdr_oarchive(unsigned int flags = 0)
	{
		xdrmem_create(&xdrs, buffer, sizeof(buffer), XDR_ENCODE);
	}

	/**
	 * Destructor
	 * Destroy XDR data structure
	 */
	~xdr_oarchive()
	{
		xdr_destroy(&xdrs);
	}

	/**
	 * Saving Archive Concept::get_library_version()
	 * @return This library's version.
	 */
	unsigned int get_library_version()
	{
		return 0;
	}

	/**
	 * Saving Archive Concept::register_type<T>() and ::register_type(u)
	 * @param The data type to register in this archive.
	 * @return
	 */
	template <class T>
	const boost::archive::detail::basic_pointer_oserializer *
	register_type(T * = NULL)
	{
		return NULL;
	}

	/**
	 * Saving Archive Concept::operator<<
	 * @param t The type to save.
	 * @return *this
	 */
	template <class T>
	xdr_oarchive &operator<<(T const &t)
	{
		return save_a_type(t, boost::mpl::bool_ <boost::serialization::implementation_level <T>::value == boost::serialization::primitive_type>());
	}

	/**
	 * Saving Archive Concept::operator&
	 * @param t The type to save.
	 * @return *this
	 */
	template <class T>
	xdr_oarchive &operator&(T const &t)
	{
		return this->operator<<(t);
	}

	// archives are expected to support this function
	void save_binary(const void *address, std::size_t count)
	{
		if (!xdr_opaque(&xdrs, address, count))
			THROW_SAVE_EXCEPTION;
	}

	/**
	 * Specialisation for writing out composite types (objects).
	 * @param t a serializable class or struct.
	 * @return *this
	 */
	template <class T>
	typename boost::disable_if <boost::is_array <T>, xdr_oarchive &>::type
	save_a_type(T const &t, boost::mpl::false_)
	{
#if BOOST_VERSION >=104100
		boost::archive::detail::save_non_pointer_type<xdr_oarchive<> >::save_only::invoke(*this, t);
#else
		boost::archive::detail::save_non_pointer_type<xdr_oarchive<>, T>::save_only::invoke(*this, t);
#endif
		return *this;
	}

	/**
	 * Specialisation for writing out composite types (C-style arrays).
	 * @param t a serializable array
	 * @return *this
	 */
	template <class T, int N>
	xdr_oarchive &save_a_type(T const(&t)[N], boost::mpl::false_)
	{
		for (int i = 0; i < N; ++i) {
			*this << t[i];
		}
		return *this;
	}

	/**
	 * Get the size of XDR representation
	 * @return size of XDR representation
	 */
	std::size_t getArchiveSize(void) const
	{
		return ((std::size_t) xdr_getpos((XDR *) &xdrs));
	}

	/**
	 * Get the XDR buffer
	 * @return pointer to XDR buffer
	 */
	const char * get_buffer(void) const
	{
		return buffer;
	}
};

// required by export
BOOST_SERIALIZATION_REGISTER_ARCHIVE( xdr_oarchive <> )

#endif // XDR_OARCHIVE_HPP
