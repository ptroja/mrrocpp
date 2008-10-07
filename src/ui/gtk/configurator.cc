#include <libxml/parser.h>
#include <libxml/tree.h>
#include <libxml/xinclude.h>
#include <libxml/xmlIO.h>
#include <libxml/xpath.h>

#include <gtk/gtktreestore.h>

#include "configurator.h"
#include <stdio.h>
#include <strings.h>
#include <string.h>

class configurator *config;

xmlXPathObjectPtr
configurator::getnodeset (xmlDocPtr doc, const xmlChar *xpath){

	xmlXPathContextPtr context;
	xmlXPathObjectPtr result;

	context = xmlXPathNewContext(doc);
	if (context == NULL) {
		printf("Error in xmlXPathNewContext\n");
		return NULL;
	}
	result = xmlXPathEvalExpression(xpath, context);
	xmlXPathFreeContext(context);
	if (result == NULL) {
		printf("Error in xmlXPathEvalExpression\n");
		return NULL;
	}
	if(xmlXPathNodeSetIsEmpty(result->nodesetval)){
		xmlXPathFreeObject(result);
                printf("No result\n");
		return NULL;
	}
	return result;
}

int configurator::populate_tree_model()
{
	{
		xmlXPathObjectPtr mp = getnodeset(this->doc, (xmlChar *) "/config/mp[@active='true']");
		if (mp) {
			xmlNodeSetPtr nodeset = mp->nodesetval;
			for (int i = 0; i < nodeset->nodeNr; i++) {
				// get name attribute of the sensors
				xmlChar *mp_name = xmlGetProp(nodeset->nodeTab[i], (xmlChar *) "name");
				if (!mp_name)
					continue;

				xmlChar query[256];

				sprintf((char *) query, "/config/mp[@name='%s']/program_name", mp_name);
				char *program_name = this->get_string(query);

				sprintf((char *) query, "/config/mp[@name='%s']/node_name", mp_name);
				char *node_name = this->get_string(query);

				GtkTreeIter parent, child;
				gtk_tree_model_get_iter_from_string(GTK_TREE_MODEL(this->store), &parent, "0");

				gtk_tree_store_append(this->store, &child, &parent);
				gtk_tree_store_set(store, &child, NAME_COLUMN, program_name, NODE_NAME_COLUMN, node_name,
						IS_RUNNING_COLUMN, FALSE, -1);
			}
			xmlXPathFreeObject(mp);
		}
	}

	{
		xmlXPathObjectPtr active_sensors = getnodeset(this->doc, (xmlChar *) "/config/sensors/sensor[@active='true']");
		if (active_sensors) {
			xmlNodeSetPtr nodeset = active_sensors->nodesetval;
			for (int i = 0; i < nodeset->nodeNr; i++) {
				// get name attribute of the sensors
				xmlChar *sensor_name = xmlGetProp(nodeset->nodeTab[i], (xmlChar *) "name");
				if (!sensor_name)
					continue;

				xmlChar query[256];

				sprintf((char *) query, "/config/sensors/sensor[@name='%s']/vsp/program_name", sensor_name);
				char *program_name = this->get_string(query);

				sprintf((char *) query, "/config/sensors/sensor[@name='%s']/vsp/node_name", sensor_name);
				char *node_name = this->get_string(query);

				GtkTreeIter parent, child;
				gtk_tree_model_get_iter_from_string(GTK_TREE_MODEL(this->store), &parent, "1");

				gtk_tree_store_append(this->store, &child, &parent);
				gtk_tree_store_set(store, &child, NAME_COLUMN, program_name, NODE_NAME_COLUMN, node_name,
						IS_RUNNING_COLUMN, FALSE, -1);
			}
			xmlXPathFreeObject(active_sensors);
		}
	}

	{
		xmlXPathObjectPtr active_effectors = getnodeset(this->doc,
				(xmlChar *) "/config/effectors/effector[@active='true']");
		if (active_effectors) {
			xmlNodeSetPtr nodeset = active_effectors->nodesetval;
			for (int i = 0; i < nodeset->nodeNr; i++) {
				// get name attribute of the sensors
				xmlChar *effector_name = xmlGetProp(nodeset->nodeTab[i], (xmlChar *) "name");
				if (!effector_name)
					continue;

				xmlChar query[256];

				sprintf((char *) query, "/config/effectors/effector[@name='%s']/ecp/program_name", effector_name);
				char *program_name = this->get_string(query);

				sprintf((char *) query, "/config/effectors/effector[@name='%s']/ecp/node_name", effector_name);
				char *node_name = this->get_string(query);

				GtkTreeIter parent, child;
				gtk_tree_model_get_iter_from_string(GTK_TREE_MODEL(this->store), &parent, "2");

				gtk_tree_store_append(this->store, &child, &parent);
				gtk_tree_store_set(store, &child, NAME_COLUMN, program_name, NODE_NAME_COLUMN, node_name,
						IS_RUNNING_COLUMN, FALSE, -1);

				sprintf((char *) query, "/config/effectors/effector[@name='%s']/edp/program_name", effector_name);
				program_name = this->get_string(query);
				if (program_name) {

					sprintf((char *) query, "/config/effectors/effector[@name='%s']/edp/node_name", effector_name);
					node_name = this->get_string(query);

					parent = child;
					gtk_tree_store_append(this->store, &child, &parent);
					gtk_tree_store_set(store, &child, NAME_COLUMN, program_name, NODE_NAME_COLUMN, node_name,
							IS_RUNNING_COLUMN, FALSE, -1);
				}
			}
			xmlXPathFreeObject(active_effectors);
		}
	}
}

configurator::configurator()
{
	// get the [configs/xml] subdirectory
	getcwd(config_dir, sizeof(config_dir));

	char *ptr;
	for (int i = 0; i < 3; i++) {
		ptr = rindex(config_dir, '/');
		*ptr = '\0';
	}
	strcat(config_dir, "/configs/xml");

	// create TreeView model
	store = gtk_tree_store_new(N_COLUMNS, G_TYPE_STRING, G_TYPE_STRING, G_TYPE_BOOLEAN);

	GtkTreeIter iter1; /* Parent iter */
	GtkTreeIter iter2; /* Child iter  */

	const char *main_rows[] = {
		"Master Process",
		"Sensors",
		"Effectors"
	};

	for (int i = 0; i < N_ROWS; i++) {
		gtk_tree_store_append(store, &iter1, NULL); /* Acquire a top-level iterator */
		gtk_tree_store_set(store, &iter1, NAME_COLUMN, main_rows[i], IS_RUNNING_COLUMN, FALSE, -1);
	}
}

int configurator::open_config_file(const char *filename)
{
	// parse config file
	if (doc) {
		xmlFreeDoc(doc);
	}
	doc = xmlReadFile(filename, NULL, XML_PARSE_XINCLUDE);
	if (doc == NULL) {
		fprintf(stderr, "failed to parse the including file\n");
		exit(1);
	}

	// apply the XInclude process
	if (xmlXIncludeProcess(doc) < 0) {
		fprintf(stderr, "XInclude processing failed\n");
		exit(1);
	}

	this->populate_tree_model();
}

configurator::~configurator()
{
	// free the document
	if (doc) {
		xmlFreeDoc(doc);
	}

	// Cleanup function for the XML library.
	xmlCleanupParser();
}

char * configurator::get_string(const xmlChar *xpath)
{
	xmlXPathObjectPtr result = getnodeset(this->doc, xpath);
	if (!result) return NULL;
	if (result->type != XPATH_NODESET) {
		xmlXPathFreeObject(result);
		return NULL;
	}

	char *ret = (char *) xmlNodeListGetString(this->doc, result->nodesetval->nodeTab[0]->children, 1);
	xmlXPathFreeObject(result);
	return ret;
}
