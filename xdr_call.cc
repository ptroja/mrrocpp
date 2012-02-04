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

#include <boost/bind.hpp>
#include <boost/function.hpp>

#include <boost/type_traits/function_traits.hpp>

std::string buffer;

class iface {
private:
	std::string id;

public:
	template<typename T>
	iface(const T * me)
	{
		id = (boost::units::detail::demangle(typeid(T).name()));
		std::cout << id << std::endl;
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
	template<typename... Args>
	struct dummy {
		dummy() {
			std::cout << "dummy(" << sizeof...(Args) << ")" << std::endl;
		}

		~dummy() {
			std::cout << "~dummy(" << sizeof...(Args) << ")" << std::endl;
		}

	};
public:
	foo_iface() : iface(this) {
	}

	virtual void operator()(int a, int b, int c, int d) = 0;

	virtual ~foo_iface() {};

	template <typename F, typename... Fargs>
	void
	untuple(boost::archive::text_iarchive & ia, F & f, const dummy< > &&, Fargs... fargs)
	{
		std::cout << "Final." << std::endl;

		(this ->* f)(fargs...);
	}

	template <typename F, typename Targ, typename... Targs, typename... Fargs>
	void
	untuple(boost::archive::text_iarchive & ia, F & f, const dummy<Targ, Targs...> &&, Fargs... fargs)
	{
		std::cout << "Partial." << std::endl;

		std::cout
			<< "untuple(" << boost::units::detail::demangle(typeid(Targ).name())
			<< ") = " << std::endl;

		Targ t;
		ia >> t;

		untuple(ia, f, dummy<Targs...>(), fargs..., t);
	}


	template<typename C, typename... Args>
	void
	call_me(boost::archive::text_iarchive & ia, void (C::* op)(Args...))
	{
		std::cout << "operator()::arity = " << boost::function_traits<void(Args...)>::arity << std::endl;

		untuple(ia, op, dummy<Args...>() );
	}

	void
	call(boost::archive::text_iarchive & ia)
	{
		call_me(ia, &foo_iface::operator());
	}
};

class foo : public foo_iface {
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

		foo fo;

		fo.call(ia);
	}

	return 0;
}



