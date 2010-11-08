#ifndef __UI_WIDGET_ENTRY_H
#define __UI_WIDGET_ENTRY_H

#include <iostream>
#include <vector>

#include <gtk/gtkbuilder.h>
#include <gtk/gtk.h>

class ui_widget_entry
{
	public:

		ui_widget_entry();
		ui_widget_entry(const char *ui_def = NULL);
		~ui_widget_entry();

		void copy(ui_widget_entry &entry);

		void ListBuilderObjects(void);

		GtkBuilder & getBuilder(void)
		{
			if (this->builder)
			{
				return *(GTK_BUILDER(this->builder));
			}
			else
			{
				throw ("no GtkBuilder in this widget");
			}
		}

	private:

		GtkBuilder *builder;
		GtkWidget *window;
		GModule *module;
};

#endif /* __UI_CONFIG_ENTRY_H */
