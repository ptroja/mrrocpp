#if !defined(_CONDITION_H_)
#define _CONDITION_H_

#include <string>
#include <list>

#include "base/lib/configurator.h"

namespace mrrocpp {
namespace mp {
namespace common {

class Condition
{
	public:
		enum RELATIONAL_OPERATOR {EQUAL_TO, NOT_EQUAL, LESS_EQUAL, GREATER_EQUAL, LESS_THAN, GREATER_THAN, WITHOUT_OP};

	public:
		Condition(const std::string & condDesc, const lib::configurator &_config);

		bool checkCompareResult();
		bool checkContext(const std::string & toCheck);
		void setResult(bool result);
		static std::list<std::string> returnSplitedStr(const std::string & toSplit);
		const std::string & getCondDesc() const;
		RELATIONAL_OPERATOR splitCondExpr();

	private:
		const std::string condition;
		std::string lhValue;
		std::string rhValue;
		bool result;
		RELATIONAL_OPERATOR operationType;

		const lib::configurator &config;
};

} // namespace common
} // namespace mp
} // namespace mrrocpp


#endif
