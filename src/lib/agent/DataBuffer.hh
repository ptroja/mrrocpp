#ifndef _DATABUFFER_HH
#define _DATABUFFER_HH

#include "Agent.hh"

class DataBufferBase {
protected:
	const std::string name;
public:
	DataBufferBase(const std::string & _name)
		: name(_name)
	{
	}

	const std::string & getName() const
	{
		return name;
	}

	//! This is required to make a class polimorphic
	virtual ~DataBufferBase() {};
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
