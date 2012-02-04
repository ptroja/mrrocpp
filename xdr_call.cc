/*
 * xdr_call.cc
 *
 *  Created on: Feb 3, 2012
 *      Author: ptroja
 */

#include <iostream>
#include <sstream>

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>

#include <boost/units/detail/utility.hpp>

std::string buffer;

class iface {
private:
	std::string id;

private:
	template<typename... Args>
	struct dummy {
	};

	template<typename C, typename... Args>
	std::size_t arity(void (C::* op)(Args...)) const {
		return sizeof...(Args);
	}

	template<typename C, typename... Args>
	C * caller_cast(void (C::* op)(Args...)) {
		return dynamic_cast<C *>(this);
	};

	template <typename F, typename... Fargs>
	void
	unpack(boost::archive::text_iarchive & ia, F & f, const dummy< > &&, Fargs&... fargs)
	{
		std::cout << "Final: " << sizeof...(Fargs) << "/" << arity(f) << std::endl;

		(caller_cast(f) ->* f)(fargs...);
	}

	template <typename F, typename Targ, typename... Targs, typename... Fargs>
	void
	unpack(boost::archive::text_iarchive & ia, F & f, const dummy<Targ, Targs...> &&, Fargs&... fargs)
	{
		std::cout << "Partial: " << sizeof...(Fargs) << "/" << arity(f) << std::endl;

		Targ t;
		ia >> t;

		std::cout
			<< "unpack(" << boost::units::detail::demangle(typeid(Targ).name())
			<< ") = " << t << std::endl;

		unpack(ia, f, dummy<Targs...>(), fargs..., t);
	}

public:
	template<typename T>
	iface(const T * me)
	{
		id = (boost::units::detail::demangle(typeid(T).name()));
		std::cout << id << std::endl;
	}

	virtual ~iface() {};

	template<typename C, typename... Args>
	void
	call_me(boost::archive::text_iarchive & ia, void (C::* op)(Args...))
	{
		std::cout << "operator()::arity = " << arity(op) << std::endl;

		unpack(ia, op, dummy<Args...>() );
	}
};

class stub {
private:
	template<typename T>
	void operator()(boost::archive::text_oarchive & oa, const T& value) {
		oa << value;
	}

	template<typename T, typename... Args>
	void operator()(boost::archive::text_oarchive & oa, const T& value, const Args&... args) {
		oa << value;
		this->operator()(oa, args...);
	}

public:
	template<typename... Args>
	void operator()(const Args&... args) {
		std::ostringstream os;

		boost::archive::text_oarchive oa(os);

		this->operator()(oa, args...);

		std::cout << os.str() << std::endl;

		buffer = os.str();
	}

};


template<typename C, typename... Args>
struct helper {
	typedef C class_type;
};

class foo_iface : public iface {
public:
	foo_iface() : iface(this) {
	}

	virtual void operator()(int a, int b, int c, int d) = 0;
};

class foo_impl : public foo_iface {
public:
	void operator()(int a, int b, int c, int d) {
		std::cout << a << "+" << b << "+" << c << "+" << d << std::endl;
	}
};

class foo_stub : public foo_iface, private stub {
public:
	void operator()(int a, int b, int c, int d) {
		stub::operator()(a, b, c, d);
	}

};

int main()
{
	{
		foo_stub fs;

		fs(1, 2, 3, 4);
	}

	std::cout << "buffer is: " << buffer << std::endl;

	{
		std::istringstream is(buffer);

		boost::archive::text_iarchive ia(is);

		foo_impl fo;

		fo.call_me(ia, &foo_iface::operator());
	}

	return 0;
}



