#ifndef XDR_OARCHIVE_HPP
#define XDR_OARCHIVE_HPP

#include <boost/archive/detail/common_oarchive.hpp>

#include <rpc/rpc.h>

/////////////////////////////////////////////////////////////////////////
// class trivial_oarchive - read serialized objects from a input text stream
template <std::size_t size=4096>
class xdr_oarchive : 
    public boost::archive::detail::common_oarchive<xdr_oarchive<size> >
{
    typedef xdr_oarchive derived_t;
    typedef boost::archive::detail::common_oarchive<xdr_oarchive<size> > base_t;

    // permit serialization system privileged access to permit
    // implementation of inline templates for maximum speed.
    friend class boost::archive::save_access;

    // member template for saveing primitive types.
    // Override for any types/templates that special treatment
    //template<class T>
    //void save(T t);

    void save(const bool t) {
        if(!xdr_bool(&xdrs, (bool_t *) &t)) throw;
    }

    void save(const char t) {
        if(!xdr_char(&xdrs, (char *)&t)) throw;
    }

    void save(const double t) {
        if(!xdr_double(&xdrs, (double *)&t)) throw;
    }

    void save(const float t) {
        if(!xdr_float(&xdrs, (float *)&t)) throw;
    }

    void save(const int t) {
        if(!xdr_int(&xdrs, (int *)&t)) throw;
    }

    void save(const long t) {
        if(!xdr_long(&xdrs, (long *)&t)) throw;
    }

    void save(const short t) {
        if(!xdr_short(&xdrs, (short *)&t)) throw;
    }

    void save(const unsigned char t) {
        if(!xdr_u_char(&xdrs, (unsigned char *)&t)) throw;
    }

    void save(const unsigned int t) {
        if(!xdr_u_int(&xdrs, (unsigned int *)&t)) throw;
    }

    void save(const unsigned long t) {
        if(!xdr_u_long(&xdrs, (unsigned long *)&t)) throw;
    }

    void save(const unsigned short t) {
        if(!xdr_u_short(&xdrs, (unsigned short *)&t)) throw;
    }

    void save(const boost::archive::class_name_type & t){
        // TODO
    }

    void save(const std::string & t) {
    	char * p = (char *) t.c_str();
    	if(!xdr_wrapstring(&xdrs, &p)) throw;
    }

private:
    char buffer[size];
    XDR xdrs;

public:
    //////////////////////////////////////////////////////////
    // public interface used by programs that use the
    // serialization library

    // archives are expected to support this function
    void save_binary(const void *address, std::size_t count) {
    	if(!xdr_opaque(&xdrs, address, count)) throw;
    }

    xdr_oarchive(unsigned int flags = 0) :
        base_t(flags)
    {
        xdrmem_create(&xdrs, buffer, sizeof(buffer), XDR_ENCODE);
    }

    ~xdr_oarchive() {
        xdr_destroy(&xdrs);
    }

    std::size_t get_size(void) const {
    	return ((std::size_t) xdr_getpos(&xdrs));
    }

    const char * get_buffer(void) const {
    	return buffer;
    }
};

// required by export
BOOST_SERIALIZATION_REGISTER_ARCHIVE(xdr_oarchive<>)

#endif // XDR_OARCHIVE_HPP
