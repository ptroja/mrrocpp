/**
 * \file xdr_iarchive.hpp
 *
 * \brief XDR real-time input boost::archive
 *
 * \author Piotr Trojanek <piotr.trojanek@gmail.com>
 */

#ifndef XDR_IARCHIVE_HPP
#define XDR_IARCHIVE_HPP

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/is_bitwise_serializable.hpp>
#include <boost/archive/detail/iserializer.hpp>
#include <boost/archive/archive_exception.hpp>
#include <boost/archive/detail/register_archive.hpp>
#include <boost/config.hpp>
#include <boost/utility/enable_if.hpp>
#include <boost/type_traits/is_enum.hpp>
#include <boost/type_traits/is_array.hpp>
#include <boost/mpl/bool.hpp>
#include <boost/version.hpp>

#include <cstring>

#include <rpc/rpc.h>

#if BOOST_VERSION >104200
#define BOOST_IARCHIVE_EXCEPTION input_stream_error
#else
#define BOOST_IARCHIVE_EXCEPTION stream_error
#endif

#define THROW_LOAD_EXCEPTION \
	boost::serialization::throw_exception( \
			boost::archive::archive_exception( \
				boost::archive::archive_exception::BOOST_IARCHIVE_EXCEPTION))

#define LOAD_A_TYPE(T, P) \
    /** conversion for T */ \
    xdr_iarchive &load_a_type(T &t,boost::mpl::true_) { \
        if(!P(&xdrs, (T *) &t)) THROW_LOAD_EXCEPTION; \
        return *this; \
    } \
    \
    /** conversion for T[] */ \
    template<int N> \
    xdr_iarchive &load_a_type(T (&t)[N],boost::mpl::false_) { \
        if(!xdr_vector(&xdrs, (char *)t, N, sizeof(T), (xdrproc_t) P)) THROW_LOAD_EXCEPTION; \
        return *this; \
    }

template <std::size_t size = 16384>
class xdr_iarchive
{
private:
    char buffer[size];
    XDR xdrs;

public:
    //! conversion for bool, special since bool != bool_t
    xdr_iarchive &load_a_type(bool &t,boost::mpl::true_) {
    	bool_t b;
        if(!xdr_bool(&xdrs, &b)) THROW_LOAD_EXCEPTION;
        t = (b) ? true : false;
        return *this;
    }

    //! conversion for bool[], special since bool != bool_t
    /* @bug: this is probably buggy becasuse xdr_bool works with C-style booleans (integers), see above method
    template<int N>
    xdr_iarchive &load_a_type(bool (&t)[N],boost::mpl::false_) {
        if(!xdr_vector(&xdrs, (char *)t, N, sizeof(bool), (xdrproc_t) xdr_bool)) THROW_LOAD_EXCEPTION;
        return *this;
    }
    */

    //! conversion for an enum
    template <class T>
    typename boost::enable_if<boost::is_enum<T>, xdr_iarchive &>::type
    load_a_type(T &t,boost::mpl::true_) {
        if(!xdr_enum(&xdrs, (enum_t *) &t)) THROW_LOAD_EXCEPTION;
        return *this;
    }

    //! conversion for std::string
	xdr_iarchive &load_a_type(std::string & t,boost::mpl::true_) {
    	char b[size];
    	char * p = b;
    	if(!xdr_wrapstring(&xdrs, &p)) THROW_LOAD_EXCEPTION;
    	t = p;
    	return *this;
    }

	//! conversion for 'char *' string
	xdr_iarchive &load_a_type(char *t, boost::mpl::false_)
	{
		if (!xdr_wrapstring(&xdrs, (char **)&t))
			THROW_LOAD_EXCEPTION;
		return *this;
	}

    LOAD_A_TYPE(char, xdr_char)
    LOAD_A_TYPE(double, xdr_double)
	LOAD_A_TYPE(float, xdr_float)
	LOAD_A_TYPE(int, xdr_int)
	LOAD_A_TYPE(long, xdr_long)
	LOAD_A_TYPE(short, xdr_short)
	LOAD_A_TYPE(unsigned char, xdr_u_char)
	LOAD_A_TYPE(unsigned int, xdr_u_int)
	LOAD_A_TYPE(unsigned long, xdr_u_long)
	LOAD_A_TYPE(unsigned short, xdr_u_short)

//	LOAD_A_TYPE(int16_t, xdr_int16_t)
//	LOAD_A_TYPE(uint16_t, xdr_u_int16_t)
//	LOAD_A_TYPE(int32_t, xdr_int32_t)
//	LOAD_A_TYPE(uint32_t, xdr_u_int32_t)
	LOAD_A_TYPE(int64_t, xdr_int64_t)
#if defined(__QNXNTO__) || (defined(__APPLE__) && defined(__MACH__))
	LOAD_A_TYPE(uint64_t, xdr_u_int64_t)
#else
	LOAD_A_TYPE(uint64_t, xdr_uint64_t)
#endif

    /**
     * Saving Archive Concept::is_loading
     */
    typedef boost::mpl::bool_<true> is_loading;

    /**
     * Saving Archive Concept::is_saving
     */
    typedef boost::mpl::bool_<false> is_saving;

    /**
     * Constructor
     * @param _buffer data buffer
     * @param _buffer_size data buffer size
     * @return
     */
    xdr_iarchive(const char * _buffer, std::size_t _buffer_size)
    {
    	assert(_buffer_size <= size);
    	std::memcpy(buffer, _buffer, _buffer_size);
        xdrmem_create(&xdrs, buffer, sizeof(buffer), XDR_DECODE);
    }

    /**
     * Destructor
     */
    ~xdr_iarchive() {
        xdr_destroy(&xdrs);
    }

    /**
     * Loading Archive Concept::get_library_version()
     * @return This library's version.
     */
    unsigned int get_library_version() { return 0; }

    /**
     * Loading Archive Concept::reset_object_address(v,u)
     * @param new_address
     * @param old_address
     */
    void reset_object_address(const void * new_address, const void * old_address) {}

    /**
     * Loading Archive Concept::delete_created_pointers()
     */
    void delete_created_pointers() {}

    /**
     * Loading Archive Concept::register_type<T>() and ::register_type(u)
     * @param The data type to register in this archive.
     * @return
     */
    template<class T>
    const boost::archive::detail::basic_pointer_iserializer *
    register_type(T * = NULL) {return 0;}

    /**
     * The standard type loading function. It forwards any type T
     * to the correct internal load_a_type function.
     */
    template<class T>
    void load_override(T & t, BOOST_PFTO int){
        load_a_type(t, boost::mpl::bool_<boost::serialization::implementation_level<T>::value == boost::serialization::primitive_type>() );
    }

    /**
     * Specialisation for reading in composite types (objects).
     * @param t a serializable class or struct.
     * @return *this
     */
    template<class T>
    typename boost::disable_if<boost::is_array<T>, xdr_iarchive &>::type
    load_a_type(T &t,boost::mpl::false_){
#if BOOST_VERSION >=104100
    	boost::archive::detail::load_non_pointer_type<xdr_iarchive<> >::load_only::invoke(*this,t);
#else
    	boost::archive::detail::load_non_pointer_type<xdr_iarchive<>,T>::load_only::invoke(*this,t);
#endif
        return *this;
    }

    /**
     * Specialisation for reading in composite types (C-style arrays).
     * @param t a serializable array
     * @return *this
     */
    template<class T, int N>
    xdr_iarchive &load_a_type(T (&t)[N],boost::mpl::false_){
    	for(int i = 0; i < N; ++i) {
    		*this >> t[i];
    	}
    	return *this;
    }

    /**
     * These load_override functions are required to handle the nvt<T> cases in the
     * serialization code. GCC won't compile that code without these overloads.
     * Others may be required as well and may need to be added later on.
     */
#if 0
    void load_override(const boost::serialization::nvp<boost::serialization::collection_size_type> & t, int){
         size_t x=0;
         * this >> x;
         t.value() = boost::serialization::collection_size_type(x);
     }
#endif
    template<class T>
    void load_override(const boost::serialization::nvp<T> & t, int){
         T& x(t.value());
         * this >> x;
    }

    /**
     * Loading Archive Concept::operator>>
     * @param t The type to load.
     * @return *this
     */
    template<class T>
    xdr_iarchive &operator>>(T &t){
        this->load_override(t, 0);
        return * this;
    }

    /**
     * Loading Archive Concept::operator&
     * @param t The type to load.
     * @return *this
     */
    template<class T>
    xdr_iarchive &operator&(T &t){
        return this->operator>>(t);
    }

    // archives are expected to support this function
    void load_binary(void *address, std::size_t count) {
    	if(!xdr_opaque(&xdrs, address, count)) THROW_LOAD_EXCEPTION;
    }

    std::size_t getArchiveSize(void) const {
    	return ((std::size_t) xdr_getpos(&xdrs));
    }
};

// required by export
BOOST_SERIALIZATION_REGISTER_ARCHIVE(xdr_iarchive<>)

#endif // XDR_IARCHIVE_HPP
