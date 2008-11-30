#ifndef __CONFIGURATOR_H

#include <libxml/tree.h>
#include <libxml/xpath.h>

#include <gtk/gtkbuilder.h>
#include <gtk/gtkstyle.h>

#include <unistd.h>

#include <iostream>
#include <map>

class configurator
{
	private:
		// XML config document
		xmlDocPtr doc;

		// configuration files directory
		char config_dir[PATH_MAX];

		xmlXPathObjectPtr getnodeset(xmlDocPtr doc, const xmlChar *xpath);

		void populate_tree_model_with_mp();
		void populate_tree_model_with_sensors();
		void populate_tree_model_with_effectors();
		void populate_tree_model();

	public:

		configurator();
		~configurator();

		int open_config_file(const char *filename);

		char *get_string(const char *xpath, ...);

		const char *getConfig_dir() const
		{
			return config_dir;
		}
};

extern class configurator *config;

#endif /* __CONFIGURATOR_H */
