
#if !defined(_CONDITION_H_)
#define _CONDITION_H_

#include <list>

#include "base/lib/configurator.h"

namespace mrrocpp {
namespace mp {
namespace common {


class Condition
{
	public:
		enum RELATIONAL_OPERATOR {EQUAL_TO = 0, NOT_EQUAL, LESS_EQUAL, GREATER_EQUAL, LESS_THAN, GREATER_THAN, WITHOUT_OP};
	public:
		Condition();
		Condition(const char * condDesc, lib::configurator &_config);
		Condition(const Condition &cond);
		~Condition();

		bool checkCompareResult();
		bool checkContext(const char *toCheck);
		void setResult(bool result);
		std::list<const char *> * returnSplitedStr(const char *toSplit);
		const char * getCondDesc() const;
		RELATIONAL_OPERATOR splitCondExpr();

	private:
		char *condition;
		char *lhValue;
		char *rhValue;
		bool result;
		RELATIONAL_OPERATOR operationType;

		lib::configurator &config;
};

} // namespace common
} // namespace mp
} // namespace mrrocpp


#endif
