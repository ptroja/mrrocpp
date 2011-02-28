#include <string>
#include <cstring>
#include <cstdlib>

#include "Condition.h"

namespace mrrocpp {
namespace mp {
namespace common {


Condition::Condition(const std::string & _condDesc, lib::configurator &_config)
	: condition(_condDesc), config(_config)
{
	operationType = splitCondExpr();
//	checkContext(condition);
//	char *fileName = config.value<std::string>("xml_file", "[xml_settings]");
//	printf("from config: %s\n", fileName);
}

Condition::RELATIONAL_OPERATOR Condition::splitCondExpr()
{
	const char *opc[] = {"==", "!=", "<=", ">=", "<", ">"};
	char *temp;

	// TODO: memory leak
	char *myExpr = strdup(condition.c_str());

	if(myExpr != NULL)
	{
		for(int i=0; i<6; i++)
		{
			char *res;

			if((res = strstr(myExpr, opc[i])) != NULL)
			{
				if(strlen(opc[i]) == 2)
					strncpy(res, "  ", 2);
				else
					strncpy(res, " ", 1);
				temp = strtok(myExpr, " ");
				lhValue = temp;
				temp = strtok(NULL, " ");
				rhValue = temp;
				//printf("lv: %s\nrv: %s\n", lhValue, rhValue);
				return (Condition::RELATIONAL_OPERATOR)i;
			}
		}
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
		bool result = checkContext(condition);
		return result;
	}
	else
		return false;
}

bool Condition::checkContext(const std::string & toCheck)
{
	if(strstr(toCheck.c_str(), ".")!=NULL)
	{
		std::list<const char *> args = returnSplitedStr(toCheck);
		std::list<const char *>::iterator it = args.begin();
		if(!strcmp("iniFile", (*it)))
		{
			if(config.exists(*(++it)))
				return (bool)config.value<int>(*it);
			return false;
		}
		//for(std::list<char *>::iterator it = args->begin(); it != args->end(); ++it)
		//	printf("Element: %s\n", (*it));
	}
	return false;
}

std::list<const char *> Condition::returnSplitedStr(const std::string & toSplit)
{
	// TODO: memory leak - change from char * to std::string implementation
	char *dataStr = strdup(toSplit.c_str());
	char *element;
	std::list<const char *> splitedStr;

	element = strtok(dataStr, ".");
	splitedStr.push_back(element);
	while((element = strtok(NULL, ".")) != NULL)
		splitedStr.push_back(element);

	return splitedStr;
}

void Condition::setResult(bool result)
{
	this->result = result;
}


} // namespace common
} // namespace mp
} // namespace mrrocpp

