#ifndef XDR_IARCHIVE_HPP
#define XDR_IARCHIVE_HPP

#include <boost/archive/detail/common_iarchive.hpp>

#include <rpc/rpc.h>
#include <cstring>

/////////////////////////////////////////////////////////////////////////
// class trivial_iarchive - read serialized objects from a input text stream
template <std::size_t size=4096>
class xdr_iarchive : 
    public boost::archive::detail::common_iarchive<xdr_iarchive<size> >
{
    typedef xdr_iarchive derived_t;
    typedef boost::archive::detail::common_iarchive<xdr_iarchive<size> > base_t;

    // permit serialization system privileged access to permit
    // implementation of inline templates for maximum speed.
    friend class boost::archive::load_access;

    // member template for loading primitive types.
    // Override for any types/templates that special treatment
    //template<class T>
    //void load(T & t);

    void load(bool & t) {
        if(!xdr_bool(&xdrs, (bool_t *) &t)) throw;
    }

    void load(char & t) {
        if(!xdr_char(&xdrs, &t)) throw;
    }

    void load(double & t) {
        if(!xdr_double(&xdrs, &t)) throw;
    }

    void load(float & t) {
        if(!xdr_float(&xdrs, &t)) throw;
    }

    void load(int & t) {
        if(!xdr_int(&xdrs, &t)) throw;
    }

    void load(long & t) {
        if(!xdr_long(&xdrs, &t)) throw;
    }

    void load(short & t) {
        if(!xdr_short(&xdrs, &t)) throw;
    }

    void load(unsigned char & t) {
        if(!xdr_u_char(&xdrs, &t)) throw;
    }

    void load(unsigned int & t) {
        if(!xdr_u_int(&xdrs, &t)) throw;
    }

    void load(unsigned long & t) {
        if(!xdr_u_long(&xdrs, &t)) throw;
    }

    void load(unsigned short & t) {
        if(!xdr_u_short(&xdrs, &t)) throw;
    }

    void load(boost::archive::class_name_type & t){
        // TODO
    }

    void load(std::string & t) {
    	char b[size];
    	char * p = b;
    	if(!xdr_wrapstring(&xdrs, &p)) throw;
	t = p;
    }

private:
    char buffer[size];
    XDR xdrs;

public:
    //////////////////////////////////////////////////////////
    // public interface used by programs that use the
    // serialization library

    // archives are expected to support this function
    void load_binary(void *address, std::size_t count) {
    	if(!xdr_opaque(&xdrs, address, count)) throw;
    }

    xdr_iarchive(const char * _buffer, std::size_t _buffer_size, unsigned int flags = 0) :
        base_t(flags)
    {
	assert(_buffer_size <= size);
    	std::memcpy(buffer, _buffer, _buffer_size);
        xdrmem_create(&xdrs, buffer, sizeof(buffer), XDR_DECODE);
    }

    ~xdr_iarchive() {
        xdr_destroy(&xdrs);
    }
};

// required by export
BOOST_SERIALIZATION_REGISTER_ARCHIVE(xdr_iarchive<>)

#endif // XDR_IARCHIVE_HPP
