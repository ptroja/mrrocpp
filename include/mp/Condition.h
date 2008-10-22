
#if !defined(_CONDITION_H_)
#define _CONDITION_H_

#include <list>

#include "lib/configurator.h"

class Condition
{
	public:
		enum RELATIONAL_OPERATOR {EQUAL_TO = 0, NOT_EQUAL, LESS_EQUAL, GREATER_EQUAL, LESS_THAN, GREATER_THAN, WITHOUT_OP};
	public:
		Condition();
		Condition(char * condDesc, configurator &_config);
		Condition(const Condition &cond);
		~Condition();

		bool checkCompareResult();
		bool checkContext(char *toCheck);
		bool setResult(bool result);
		std::list<char *> * returnSplitedStr(char *toSplit);
		char * getCondDesc() const;
		RELATIONAL_OPERATOR splitCondExpr();
		
	private:
		char *condition;
		char *lhValue;
		char *rhValue;
		bool result;
		RELATIONAL_OPERATOR operationType;
		
		configurator &config;
};

#endif
