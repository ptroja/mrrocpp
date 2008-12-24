/*
 * sr_console.cc
 *
 *  Created on: Dec 24, 2008
 *      Author: ptroja
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>

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
	sr_package_t sr_msg;
	int16_t status;

	messip_channel_t *ch;
	int32_t type, subtype;
	int rcvid;

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

	gtk_tree_view_set_model (GTK_TREE_VIEW (view), GTK_TREE_MODEL(store));

	while(1)
	{
		rcvid = messip_receive(ch, &type, &subtype, &sr_msg, sizeof(sr_msg), MESSIP_NOTIMEOUT);

		if (rcvid == -1) /* Error condition, exit */
		{
			perror("SR: Receive failed\n");
			// 	  throw generator::ECP_error(SYSTEM_ERROR, (uint64_t) 0);
			break;
		} else if (rcvid < -1) {
			// ie. MESSIP_MSG_DISCONNECT
			fprintf(stderr, "ie. MESSIP_MSG_DISCONNECT\n");
			continue;
		}

		status = 0;
		messip_reply(ch, rcvid, EOK, &status, sizeof(status), MESSIP_NOTIMEOUT);

		if (strlen(sr_msg.process_name)>1) // by Y jesli ten string jest pusty to znaczy ze przyszedl smiec
		{
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

			// TODO: logowanie do pliku
		} else {
			printf("SR: unexpected message\n");
		}
	}

	return NULL;
}
