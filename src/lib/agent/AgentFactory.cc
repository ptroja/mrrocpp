#include <utility>
#include <iostream>

#include <boost/foreach.hpp>

#include "AgentFactory.hh"
#include "Agent.hh"

AgentFactory::agents_t AgentFactory::agents;

void AgentFactory::addAgent(Agent * agent) {
	agents.insert(std::make_pair(agent->getName(), agent));
}

Agent * AgentFactory::getAgent(const std::string & name)
{
	agents_t::iterator result = agents.find(name);
	if (result != agents.end()) {
		return result->second;
	}
	// TODO: or throw an exception?
	return NULL;
}

void AgentFactory::listAgents(void) {
	std::cout << "Agents[" << agents.size() << "]:" << std::endl;
	BOOST_FOREACH(agent_item_t item, agents) {
		std::cout << "\t" << item.first << std::endl;
	}
}
