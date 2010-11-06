/*
 * DataConditionTest.cc
 *
 *  Created on: Apr 29, 2010
 *      Author: ptroja
 */

#include <iostream>

#include <boost/typeof/typeof.hpp>

#include "DataBuffer.h"
#include "AndDataCondition.h"
#include "OrDataCondition.h"

#define TEST_TYPE(a,b)				\
{									\
	typedef BOOST_TYPEOF(a) type;	\
	BOOST_STATIC_ASSERT((boost::is_same<type, b>::value));	\
}

int
main(int argc, char *argv[])
{
	DataBuffer<bool> d1("d1"), d2("d2");
	AndDataCondition a1, a2;
	OrDataCondition o1, o2;

	TEST_TYPE((d1 & d2), AndDataCondition);
	TEST_TYPE((d1 & a2), AndDataCondition);
	//TEST_TYPE((d1 & o2), AndDataCondition);

	TEST_TYPE((a1 & d2), AndDataCondition);
	TEST_TYPE((a1 & a2), AndDataCondition);
	//TEST_TYPE((a1 & o2), AndDataCondition);

	//TEST_TYPE((o1 & d2), AndDataCondition);
	//TEST_TYPE((o1 & a2), AndDataCondition);
	//TEST_TYPE((o1 & o2), AndDataCondition);

	TEST_TYPE((d1 | d2), OrDataCondition);
	TEST_TYPE((d1 | a2), OrDataCondition);
	TEST_TYPE((d1 | o2), OrDataCondition);

	TEST_TYPE((a1 | d2), OrDataCondition);
	TEST_TYPE((a1 | a2), OrDataCondition);
	TEST_TYPE((a1 | o2), OrDataCondition);

	TEST_TYPE((o1 | d2), OrDataCondition);
	TEST_TYPE((o1 | a2), OrDataCondition);
	TEST_TYPE((o1 | o2), OrDataCondition);

	return 0;
}
