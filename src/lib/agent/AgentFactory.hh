#ifndef __AGENT_FACTORY_HH
#define __AGENT_FACTORY_HH

#include <map>
#include <string>

// forward declaration
class Agent;

class AgentFactory {
private:
	typedef std::map<const std::string, Agent *> agents_t;
	typedef agents_t::value_type agent_item_t;

	static agents_t agents;

public:
	static void addAgent(Agent * agent);
	static Agent * getAgent(const std::string & name);
	static void listAgents(void);
};

#endif /* __AGENT_FACTORY_HH */
