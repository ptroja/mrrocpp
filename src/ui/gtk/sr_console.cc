/*
 * sr_console.cc
 *
 *  Created on: Dec 24, 2008
 *      Author: ptroja
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include <iostream>
#include <fstream>

#include <gtk/gtk.h>

#include "ui_model.h"

#include "messip/messip.h"
#include "lib/srlib.h"

GtkListStore *store;

enum
{
	COL_TIMESTAMP = 0,
	COL_PROCESS_NAME,
	COL_HOST_NAME,
	COL_DESCRIPTION,
	NUM_COLS
};

void *sr_thread(void* arg)
{
	messip_channel_t *ch;

	// TODO: config->return_attach_point_name(configurator::CONFIG_SERVER, "sr_attach_point", "[ui]");
	if ((ch = messip_channel_create(NULL, "sr", MESSIP_NOTIMEOUT, 0)) == NULL) {
		perror("messip_channel_create()");
		return NULL;
	}

	store = gtk_list_store_new (NUM_COLS, G_TYPE_STRING, G_TYPE_STRING, G_TYPE_STRING, G_TYPE_STRING);

	GtkWidget *view = GTK_WIDGET(ui_model::instance().getUiGObject("sr_treeview"));
	g_assert(view);

	GtkCellRenderer *renderer;

	/* --- Column #1 --- */

	renderer = gtk_cell_renderer_text_new ();
	gtk_tree_view_insert_column_with_attributes (GTK_TREE_VIEW (view),
			-1,
			"Timestamp",
			renderer,
			"text", COL_TIMESTAMP,
			NULL);

	/* --- Column #2 --- */

	renderer = gtk_cell_renderer_text_new ();
	gtk_tree_view_insert_column_with_attributes (GTK_TREE_VIEW (view),
			-1,
			"Process",
			renderer,
			"text", COL_PROCESS_NAME,
			NULL);

	/* --- Column #3 --- */

	renderer = gtk_cell_renderer_text_new ();
	gtk_tree_view_insert_column_with_attributes (GTK_TREE_VIEW (view),
			-1,
			"Host",
			renderer,
			"text", COL_HOST_NAME,
			NULL);

	/* --- Column #4 --- */

	renderer = gtk_cell_renderer_text_new ();
	gtk_tree_view_insert_column_with_attributes (GTK_TREE_VIEW (view),
			-1,
			"Description",
			renderer,
			"text", COL_DESCRIPTION,
			NULL);

	// TODO: error class column

	gtk_tree_view_set_model (GTK_TREE_VIEW (view), GTK_TREE_MODEL(store));

	g_object_unref (store);

	//! init SR log file
	time_t time_of_day;
	char timestamp[50];

	time_of_day = time( NULL );
	strftime(timestamp, 40, "%g%m%d_%H-%M-%S", localtime( &time_of_day ) );

	std::string log_file_with_dir = std::string("../logs/");
	log_file_with_dir.append(timestamp);
	log_file_with_dir.append("_sr.log");

//	std::cout << log_file_with_dir << std::endl;

	std::ofstream log_file (log_file_with_dir.c_str(), std::ios::out);

	while(1)
	{
		sr_package_t sr_msg;
		int32_t type, subtype;

		int rcvid = messip_receive(ch, &type, &subtype, &sr_msg, sizeof(sr_msg), MESSIP_NOTIMEOUT);

		if (rcvid == -1) /* Error condition, exit */
		{
			perror("SR: Receive failed\n");
			// 	  throw generator::ECP_error(SYSTEM_ERROR, (uint64_t) 0);
			break;
		} else if (rcvid < -1) {
			// ie. MESSIP_MSG_DISCONNECT
			fprintf(stderr, "messip_receive() -> %d, ie. MESSIP_MSG_DISCONNECT\n", rcvid);
			continue;
		}

		int16_t status = 0;
		messip_reply(ch, rcvid, EOK, &status, sizeof(status), MESSIP_NOTIMEOUT);

		if (strlen(sr_msg.process_name)>1) // by Y jesli ten string jest pusty to znaczy ze przyszedl smiec
		{
			//! write to UI console
			char timestamp[16];

			strftime(timestamp, 100, "%H:%M:%S", localtime(&sr_msg.ts.tv_sec));
			sprintf(timestamp+8, ".%03ld", sr_msg.ts.tv_nsec/1000000);

			GtkTreeIter iter;
			gtk_list_store_append (store, &iter);
			gtk_list_store_set (store, &iter,
					COL_TIMESTAMP, timestamp,
					COL_PROCESS_NAME, sr_msg.process_name,
					COL_HOST_NAME, sr_msg.host_name,
					COL_DESCRIPTION, sr_msg.description,
					-1);

			//! write to logfile
			char current_line[400];

			snprintf(current_line, 100, "%-10s", sr_msg.host_name);
			strcat(current_line, "  ");
			strftime(current_line+12, 100, "%H:%M:%S", localtime(&sr_msg.ts.tv_sec));
			sprintf(current_line+20, ".%03ld   ", sr_msg.ts.tv_nsec/1000000);

			switch (sr_msg.process_type) {
				case EDP:
					strcat(current_line, "EDP: ");
					break;
				case ECP:
					strcat(current_line, "ECP: ");
					break;
				case MP:
					strcat(current_line, "MP:  ");
					break;
				case VSP:
					strcat(current_line, "VSP: ");
					break;
				case UI:
					strcat(current_line, "UI:  ");
					break;
				default:
					strcat(current_line, "???: ");
					continue;
			}

			char process_name_buffer[NAME_LENGTH+1];
			snprintf(process_name_buffer, sizeof(process_name_buffer), "%-21s", sr_msg.process_name);

			strcat(current_line, process_name_buffer);

			switch (sr_msg.message_type) {
				case FATAL_ERROR:
					strcat(current_line, "FATAL_ERROR:     ");
					break;
				case NON_FATAL_ERROR:
					strcat(current_line, "NON_FATAL_ERROR: ");
					break;
				case SYSTEM_ERROR:
					strcat(current_line, "SYSTEM_ERROR:    ");
					break;
				case NEW_MESSAGE:
					strcat(current_line, "MESSAGE:         ");
					break;
				default:
					strcat(current_line, "UNKNOWN ERROR:   ");
			};

			strcat( current_line, sr_msg.description);
			strcat( current_line, "\n" );

			log_file << current_line;
			log_file.flush();
		} else {
			printf("SR: unexpected message\n");
		}
	}

	messip_channel_delete(ch, MESSIP_NOTIMEOUT);

	return NULL;
}
