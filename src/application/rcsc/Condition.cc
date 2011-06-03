#include <string>
#include <cstring>
#include <cstdlib>

#include "Condition.h"

#include "base/lib/configurator.h"

namespace mrrocpp {
namespace mp {
namespace common {


Condition::Condition(const std::string & _condDesc, const lib::configurator &_config)
	: condition(_condDesc), config(_config)
{
	operationType = splitCondExpr();
//	checkContext(condition);
//	char *fileName = config.value<std::string>("xml_file", "[xml_settings]");
//	printf("from config: %s\n", fileName);
}

Condition::RELATIONAL_OPERATOR Condition::splitCondExpr()
{
	// TODO: rewrite with boost::tokenize
	char *myExpr = strdup(condition.c_str());

	char *ptr = myExpr;

	if(myExpr != NULL)
	{
		for(int i=0; i<6; i++)
		{
			const char *opc[] = {"==", "!=", "<=", ">=", "<", ">"};
			char *res;

			if((res = strstr(myExpr, opc[i])) != NULL)
			{
				if(strlen(opc[i]) == 2)
					strncpy(res, "  ", 2);
				else
					strncpy(res, " ", 1);
				char * temp = strtok(myExpr, " ");
				lhValue = temp;
				temp = strtok(NULL, " ");
				rhValue = temp;
				//printf("lv: %s\nrv: %s\n", lhValue, rhValue);

				// free allcated memory
				free(ptr);

				return (Condition::RELATIONAL_OPERATOR)i;
			}
		}
	}

	if(ptr) {
		// free allcated memory
		free(ptr);
	}

	return Condition::WITHOUT_OP;
}

const std::string & Condition::getCondDesc() const
{
	return condition;
}

bool Condition::checkCompareResult()
{
	if(condition == "true" || condition == "TRUE")
		return true;

	if(condition == "stateOperationResult")
		return result;

	if(strstr(condition.c_str(), ".") != NULL)
	{
		return checkContext(condition);
	}

	// defalt to false
	return false;
}

bool Condition::checkContext(const std::string & toCheck)
{
	if(strstr(toCheck.c_str(), ".")!=NULL)
	{
		std::list<std::string> args = returnSplitedStr(toCheck);
		std::list<std::string>::iterator it = args.begin();
		if((*it) == "iniFile")
		{
			++it;
			if(config.exists((*it).c_str())) {
				return (bool)config.value<int>(*it);
			}
		}
		//for(std::list<char *>::iterator it = args->begin(); it != args->end(); ++it)
		//	printf("Element: %s\n", (*it));
	}

	// default to false
	return false;
}

std::list<std::string> Condition::returnSplitedStr(const std::string & toSplit)
{
	// TODO: rewrite with boost::tokenize
	char *dataStr = strdup(toSplit.c_str());

	// pointer to the allocated memory
	char *ptr = dataStr;

	std::list<std::string> splitedStr;

	char *element = strtok(dataStr, ".");
	splitedStr.push_back(element);
	while((element = strtok(NULL, ".")) != NULL)
		splitedStr.push_back(element);

	// free allocated memory
	free(ptr);

	return splitedStr;
}

void Condition::setResult(bool result)
{
	this->result = result;
}


} // namespace common
} // namespace mp
} // namespace mrrocpp

