#ifndef _DATABUFFER_HH
#define _DATABUFFER_HH

#include <vector>
#include <ostream>

#include "../xdr_iarchive.hpp"

#include "Agent.hh"

// forward declarations
class DataBufferBase;
class AndBufferContainer;
class OrBufferContainer;

//! representation of 'And' buffer condition
class AndBufferContainer : public std::vector<const DataBufferBase *> {
	//! overloaded display operator
	friend std::ostream& operator<<(std::ostream& output, const AndBufferContainer& p);

private:
	//! flag to indicate that new message has arrived
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
	bool isNewData() const;
};

class OrBufferContainer : public std::vector<AndBufferContainer> {
	//! overloaded display operator
	friend std::ostream& operator<<(std::ostream& output, const OrBufferContainer& p);

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
};

class DataBufferBase {
	//! Agent needs an access to Store/Update methods
	friend class Agent;

protected:
	//! name of the data buffer
	const std::string name;

	//! flag for marking a new data
	bool new_data_ready;

	//! store new data
	virtual void Store(xdr_iarchive<> & ia) = 0;

	//! update buffer if new data has arrived
	virtual void Update(void) = 0;

public:
	//! Constructor
	DataBufferBase(const std::string & _name);

	//! compose 'And' condition from ('Buffer' & 'Buffer')
	AndBufferContainer operator&(DataBufferBase &op);

	//! compose 'Or' condition from ('Buffer' | 'Buffer')
	OrBufferContainer operator|(DataBufferBase &op);

	//! compose 'Or' condition from ('Buffer' | 'And')
	OrBufferContainer operator|(AndBufferContainer &op);

	//! get name of the buffer
	const std::string & getName() const;

	//! This is required to make a class polimorphic
	virtual ~DataBufferBase();

	//! check if new data has arrived
	bool isNewDataReady() const;
};

template <class T>
class DataBuffer : public DataBufferBase {
	//! Agent needs an access to Store/Update methods
	friend class Agent;

private:
	//! flag indicating that the new data has not been getted yet
	bool fresh;

	//! current data
	T data;

	//! place for keeping new data after arrive
	T new_data;

	/**
	 * Store data in the buffer
	 * @param ia input archive
	 */
	void Store(xdr_iarchive<> & ia) {
 		ia >> new_data;
 		if (new_data_ready) {
 			std::cerr << "Warning: data overwrite at buffer '" << getName() << "'" << std::endl;
 		}
		new_data_ready = true;
	}

	/**
	 * Update data from the temporary to the current data buffer
	 */
	void Update(void) {
		if(new_data_ready) {
			data = new_data;
			new_data_ready = false;
			fresh = true;
		}
	}

public:
	//! Constructor
	DataBuffer(const std::string & _name, const T & _default_value = T())
		: DataBufferBase(_name), data(_default_value),
		fresh(false)
	{
	}

	/**
	 * Get data from the buffer
	 * @return current data
	 */
	T Get() {
		fresh = false;
		return data;
	}

	/**
	 * Get data from the buffer
	 * @param item where the data will be stored
	 * @return fresh flag indicatig if this data has been already "getted"
	 */
	bool Get(T & item) {
		bool fresh_flag = fresh;
		item = data;
		fresh = false;
		return fresh_flag;
	}

	/**
	 * Check if data has ben already "getted"
	 * @return fresh flag
	 */
	bool isFresh(void) const {
		return fresh;
	}
};

#endif /* _DATABUFFER_HH */
