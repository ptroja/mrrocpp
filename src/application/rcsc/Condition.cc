
#include <cstring>
#include <cstdlib>
#include <cstdio>

#include "Condition.h"


namespace mrrocpp {
namespace mp {
namespace common {


/*Condition::Condition()
{
	this->condition = NULL;
	this->lhValue = NULL;
	this->rhValue = NULL;
}
*/

Condition::Condition(const char *condDesc, lib::configurator &_config)
	: config(_config)
{
	this->condition = NULL;
	this->lhValue = NULL;
	this->rhValue = NULL;
	int size = strlen(condDesc) + 1;
	this->condition = new char[size];
	strcpy(this->condition, condDesc);
	operationType = splitCondExpr();
//	checkContext(condition);
//	char *fileName = config.value<std::string>("xml_file", "[xml_settings]");
//	printf("from config: %s\n", fileName);
}

Condition::Condition(const Condition &cond)
	: config(cond.config)
{
	this->condition = NULL;
	this->lhValue = NULL;
	this->rhValue = NULL;
	int size = strlen(cond.condition) + 1;
	this->condition = new char[size];
	strcpy(this->condition, cond.condition);
	if(cond.lhValue != NULL)
	{
		size = strlen(cond.lhValue) + 1;
		this->lhValue = new char[size];
		strcpy(this->lhValue, cond.lhValue);
	}
	if(cond.rhValue != NULL)
	{
		size = strlen(cond.rhValue) + 1;
		this->rhValue = new char[size];
		strcpy(this->rhValue, cond.rhValue);
	}
	this->operationType = cond.operationType;
}

Condition::~Condition()
{
	if(condition != NULL)
		delete[] condition;
	if(lhValue != NULL)
		delete[] lhValue;
	if(rhValue != NULL)
		delete[] rhValue;
}

Condition::RELATIONAL_OPERATOR Condition::splitCondExpr()
{
	const char *opc[] = {"==", "!=", "<=", ">=", "<", ">"};
	char *temp;

	// TODO: memory leak
	char *myExpr = strdup(condition);
	char *res;

	if(myExpr != NULL)
	{
		for(int i=0; i<6; i++)
		{
			if((res = strstr(myExpr, opc[i])) != NULL)
			{
				if(strlen(opc[i]) == 2)
					strncpy(res, "  ", 2);
				else
					strncpy(res, " ", 1);
				temp = strtok(myExpr, " ");
				lhValue = new char[strlen(temp)];
				strcpy(lhValue, temp);
				temp = strtok(NULL, " ");
				rhValue = new char[strlen(temp)];
				strcpy(rhValue, temp);
				//printf("lv: %s\nrv: %s\n", lhValue, rhValue);
				return (Condition::RELATIONAL_OPERATOR)i;
			}
		}
	}
	return Condition::WITHOUT_OP;
}

const char * Condition::getCondDesc() const
{
	return condition;
}

bool Condition::checkCompareResult()
{
	if(!strcmp(condition, "true") || !strcmp(condition, "TRUE"))
		return true;

	if(!strcmp(condition, "stateOperationResult"))
		return result;

	if(strstr(condition, ".") != NULL)
	{
		bool result = checkContext(condition);
		return result;
	}
	else
		return false;
}

bool Condition::checkContext(const char *toCheck)
{
	const char *iniFile = "iniFile";
	if(strstr(toCheck, ".")!=NULL)
	{
		std::list<const char *> *args = returnSplitedStr(toCheck);
		std::list<const char *>::iterator it = args->begin();
		if(!strcmp(iniFile, (*it)))
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

std::list<const char *> * Condition::returnSplitedStr(const char *toSplit)
{
	// TODO: memory leak - change from char * to std::string implementation
	char *dataStr = strdup(toSplit);
	char *element;
	std::list<const char *> *splitedStr = new std::list<const char *>();

	element = strtok(dataStr, ".");
	splitedStr->push_back(element);
	while((element = strtok(NULL, ".")) != NULL)
		splitedStr->push_back(element);

	return splitedStr;
}

void Condition::setResult(bool result)
{
	this->result = result;
}


} // namespace common
} // namespace mp
} // namespace mrrocpp

