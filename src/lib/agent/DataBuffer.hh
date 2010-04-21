#ifndef _DATABUFFER_HH
#define _DATABUFFER_HH

#include <vector>

#include "../xdr_iarchive.hpp"

#include "Agent.hh"

// forward declarations
class DataBufferBase;
class AndBufferContainer;
class OrBufferContainer;

//! representation of 'And' buffer condition
class AndBufferContainer : public std::vector<const DataBufferBase *> {
private:
	bool fresh;

public:
	//! assign a single 'Buffer' condition
	AndBufferContainer & operator=(const DataBufferBase &op);

	//! compose 'And' condition from ('And' & 'Buffer')
	AndBufferContainer operator&(const DataBufferBase &op);

	//! compose 'And' condition from ('And' & 'And')
	AndBufferContainer operator&(const AndBufferContainer &op);

	//! compose 'And' condition from ('And' | 'And')
	OrBufferContainer operator|(const AndBufferContainer &op);

	//! constructor from single 'Buffer' condition
	AndBufferContainer(const DataBufferBase &op);

	//! default constructor
	AndBufferContainer();

	//! check if this condition was satisfied
	bool isFresh() const;

	//! print the condition
	void print() const;
};

class OrBufferContainer : public std::vector<AndBufferContainer> {
public:
	//! Base container data type
	typedef std::vector<AndBufferContainer> base_t;

	//! assign a single 'Buffer' condition
	OrBufferContainer & operator=(const DataBufferBase &op);

	//! assign a single 'And' condition
	//OrBufferContainer & operator=(const AndBufferContainer &op);

	//! constructor from single 'Buffer' condition
	OrBufferContainer(const DataBufferBase &op);

	//! constructor from single 'And' condition
	OrBufferContainer(const AndBufferContainer &op);

	//! default constructor
	OrBufferContainer();

	//! print the condition
	void print() const;
};

class DataBufferBase {
protected:
	//! name of the data buffer
	const std::string name;

	//! flag indicating that the new data has arrived
	bool fresh;

public:
	DataBufferBase(const std::string & _name);

	//! compose 'And' condition from ('Buffer' & 'Buffer')
	AndBufferContainer operator&(DataBufferBase &op);

	//! compose 'Or' condition from ('Buffer' | 'Buffer')
	OrBufferContainer operator|(DataBufferBase &op);

	//! compose 'Or' condition from ('Buffer' | 'And')
	OrBufferContainer operator|(AndBufferContainer &op);

	//! get name of the buffer
	const std::string & getName() const;

	bool isFresh(void) const;

	virtual void Set(xdr_iarchive<> & ia) = 0;

	//! This is required to make a class polimorphic
	virtual ~DataBufferBase();
};

template <class T>
class DataBuffer : public DataBufferBase {
private:
	T data;

public:
	DataBuffer(const std::string & _name, const T & _default_value = T())
		: DataBufferBase(_name), data(_default_value)
	{
	}

	T Get() {
		fresh = false;
		return data;
	}

	bool Get(T & item) {
		bool fresh_flag = fresh;
		item = data;
		fresh = false;
		return fresh_flag;
	}

	void Set(const T & item) {
		data = item;
		fresh = true;
	}

	void Set(xdr_iarchive<> & ia) {
 		ia >> data;
		fresh = true;
	}
};

#endif /* _DATABUFFER_HH */
