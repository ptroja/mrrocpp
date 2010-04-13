#ifndef _DATABUFFER_HH
#define _DATABUFFER_HH

#include "Agent.hh"
#include <vector>

// forward declarations
class DataBufferBase;
class AndBufferContainer;
class OrBufferContainer;

//! representation of 'And' buffer condition
class AndBufferContainer : public std::vector<const DataBufferBase *> {
private:
	//! base class typedef
	typedef std::vector<const DataBufferBase *> base_t;

public:
	//! assign a single 'Buffer' condition
	AndBufferContainer & operator=(const DataBufferBase &op);

	//! compose 'And' condition from ('And' && 'Buffer')
	AndBufferContainer & operator&&(const DataBufferBase &op);

	//! compose 'And' condition from ('And' && 'And')
	AndBufferContainer & operator&&(const AndBufferContainer &op);

	//! compose 'Or' condition from ('And' || 'Buffer')
	OrBufferContainer operator||(const DataBufferBase &op);

	//! compose 'And' condition from ('And' || 'And')
	OrBufferContainer operator||(const AndBufferContainer &op);

	//! constructor from single 'Buffer' condition
	AndBufferContainer(const DataBufferBase &op);

	//! default constructor
	AndBufferContainer();
};

class OrBufferContainer : public std::vector<AndBufferContainer> {
public:
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
};

class DataBufferBase {
protected:
	const std::string name;

public:
	DataBufferBase(const std::string & _name);

	//! compose 'And' condition from ('Buffer' && 'Buffer')
	AndBufferContainer operator&&(DataBufferBase &op);

	//! compose 'Or' condition from ('Buffer' || 'Buffer')
	OrBufferContainer operator||(DataBufferBase &op);

	//! compose 'Or' condition from ('Buffer' || 'And')
	OrBufferContainer operator||(AndBufferContainer &op);

	//! get name of the buffer
	const std::string & getName() const;

	//! This is required to make a class polimorphic
	virtual ~DataBufferBase();
};

template <class T>
class DataBuffer : public DataBufferBase {
private:
	Agent & owner;
	bool fresh;
	T data;

public:
	DataBuffer(Agent & _owner, const std::string & _name, const T & _default_value = T())
		: DataBufferBase(_name), owner(_owner), fresh(false), data(_default_value)
	{
	}

	void Get(T & item, const bool wait = true) {
		boost::unique_lock<boost::mutex> lock(owner.data_mutex);
		while (wait && !fresh) {
			owner.data_condition_variable.wait(lock);
		}
		fresh = false;
		item = data;
	}

	T Get(const bool wait = true) {
		boost::unique_lock<boost::mutex> lock(owner.data_mutex);
		while (wait && !fresh) {
				owner.data_condition_variable.wait(lock);
		}
		fresh = false;
		return data;
	}

	void Set(const T & item) {
		boost::unique_lock<boost::mutex> lock(owner.data_mutex);
		data = item;
		fresh = true;
		owner.data_condition_variable.notify_one();
	}
};

#endif /* _DATABUFFER_HH */
