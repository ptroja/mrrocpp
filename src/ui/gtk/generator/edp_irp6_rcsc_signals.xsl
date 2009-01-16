<?xml version="1.0" encoding="UTF-8"?>
<!--
MRROC++ GUI generator
EDP IRp6 RCSC window callback signals
 -->
<xsl:stylesheet version="1.0" xmlns:xsl="http://www.w3.org/1999/XSL/Transform">
<xsl:output method="text"/>

<!-- signals handling file .cc-->
<xsl:template name="irp6.edp.main.signals.cc" match="*[substring(name(),1,4)='irp6']">
<xsl:variable name="name" select="name"/>
<xsl:variable name="fullName" select="fullName"/>
<xsl:document method="text" href="../edp_{$name}_rcsc_uimodule.cc">



<xsl:text>
#include &lt;iostream&gt;
#include &lt;gtk/gtk.h&gt;
#include &lt;glib.h&gt;
#include "ui_model.h"
#include "edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>_rcsc_uimodule.h"


edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>_rcsc::edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>_rcsc(ui_config_entry &amp;entry)
{	
}

edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>_rcsc::~edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>_rcsc()
{
}

static edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>_rcsc *edp_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>;


extern "C" 
{ 
	void  on_combobox1_changed_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>(GtkComboBox *comboBox, gpointer userdata)  
	{
		ui_config_entry &amp; comboEntry = *(ui_config_entry *) userdata;
		GtkBuilder &amp; builder = (comboEntry.getBuilder());
		
		GtkScrolledWindow * scrolled = GTK_SCROLLED_WINDOW (gtk_builder_get_object(&amp;builder, "scrolledwindow1"));

		//if the child exists, destroy it
		if (gtk_bin_get_child(GTK_BIN(scrolled))!=NULL)
		{
			GtkWidget* child = gtk_bin_get_child(GTK_BIN(scrolled));
			gtk_widget_destroy(child);
		}
		
		const gchar * ChosenFile;
		gint choice;
		choice = gtk_combo_box_get_active (comboBox);

		switch (choice)
		{
		case 0: std::cout &lt;&lt; "Servo algorithm window chosen" &lt;&lt; std::endl; ChosenFile = "</xsl:text><xsl:value-of select="$name" /><xsl:text>_servo_algorithm.xml"; break;
		case 1: std::cout &lt;&lt; "Internal window chosen" &lt;&lt; std::endl; ChosenFile = "</xsl:text><xsl:value-of select="$name" /><xsl:text>_int.xml"; break;
		case 2: std::cout &lt;&lt; "Increment window chosen" &lt;&lt; std::endl; ChosenFile = "</xsl:text><xsl:value-of select="$name" /><xsl:text>_inc.xml"; break;
		case 3: std::cout &lt;&lt; "XYZ Euler ZYZ window chosen" &lt;&lt; std::endl; ChosenFile = "</xsl:text><xsl:value-of select="$name" /><xsl:text>_euler_xyz.xml"; break;
		case 4: std::cout &lt;&lt; "XYZ Angle Axis window chosen" &lt;&lt; std::endl; ChosenFile = "</xsl:text><xsl:value-of select="$name" /><xsl:text>_axis_xyz.xml"; break;
		case 5: std::cout &lt;&lt; "</xsl:text><xsl:choose><xsl:when test="$name = 'irp6p'"><xsl:text>TS Euler ZYZ</xsl:text></xsl:when><xsl:otherwise><xsl:text>TS Angle Axis</xsl:text></xsl:otherwise></xsl:choose><xsl:text> window chosen" &lt;&lt; std::endl; ChosenFile = "</xsl:text><xsl:value-of select="$name" /><xsl:text>_</xsl:text><xsl:choose><xsl:when test="$name = 'irp6p'"><xsl:text>euler</xsl:text></xsl:when><xsl:otherwise><xsl:text>axis</xsl:text></xsl:otherwise></xsl:choose><xsl:text>_ts.xml"; break;
		default: std::cout &lt;&lt; "Something is not working properly!" &lt;&lt; std::endl;
		}
		
		GtkBuilder* chosenFileBuilder = gtk_builder_new();
		GError *err = NULL;
		if (gtk_builder_add_from_file(chosenFileBuilder, ChosenFile, &amp;err) == 0) 
		{
			fprintf (stderr, "Unable to read file %s: %s\n", ChosenFile, err->message);
			g_error_free (err);

			// TODO: throw(...)
		}
		g_assert(chosenFileBuilder);
		
		gpointer symbol;
		gtk_builder_connect_signals(chosenFileBuilder, symbol);

		GtkWidget* chosenWindow = GTK_WIDGET (gtk_builder_get_object (chosenFileBuilder, "window"));
		g_assert(chosenWindow);
		
		GtkWidget* windowWithoutParent = gtk_bin_get_child(GTK_BIN(chosenWindow));
		gtk_widget_unparent(windowWithoutParent);
		
		gtk_scrolled_window_add_with_viewport (scrolled, windowWithoutParent);
		
	}	

	void ui_module_init(ui_config_entry &amp;entry) 
	{
		edp_</xsl:text><xsl:value-of select="$fullName" /><xsl:text> = new edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>_rcsc(entry);
		fprintf(stderr, "module %s loaded\n", __FILE__);
		const char * def_servo_algorithm = "</xsl:text><xsl:value-of select="$name" /><xsl:text>_servo_algorithm.xml";
		const char * def_euler_xyz = "</xsl:text><xsl:value-of select="$name" /><xsl:text>_euler_xyz.xml";
		const char * def_axis_xyz = "</xsl:text><xsl:value-of select="$name" /><xsl:text>_axis_xyz.xml";
		const char * def_inc = "</xsl:text><xsl:value-of select="$name" /><xsl:text>_inc.xml";
		const char * def_int = "</xsl:text><xsl:value-of select="$name" /><xsl:text>_int.xml";
		const char * def_ts = "</xsl:text><xsl:value-of select="$name" /><xsl:text>_</xsl:text><xsl:choose><xsl:when test="$name = 'irp6p'"><xsl:text>euler</xsl:text></xsl:when><xsl:otherwise><xsl:text>axis</xsl:text></xsl:otherwise></xsl:choose><xsl:text>_ts.xml";
		new ui_config_entry(ui_config_entry::EDP, "edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>_servo_algorithm", NULL, def_servo_algorithm);
		new ui_config_entry(ui_config_entry::EDP, "edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>_int", NULL, def_int);
		new ui_config_entry(ui_config_entry::EDP, "edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>_inc", NULL, def_inc);
		new ui_config_entry(ui_config_entry::EDP, "edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>_axis_xyz", NULL, def_axis_xyz);
		new ui_config_entry(ui_config_entry::EDP, "edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>_euler_zyz", NULL, def_euler_xyz);
		new ui_config_entry(ui_config_entry::EDP, "edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>_</xsl:text><xsl:choose><xsl:when test="$name = 'irp6p'"><xsl:text>euler</xsl:text></xsl:when><xsl:otherwise><xsl:text>axis</xsl:text></xsl:otherwise></xsl:choose><xsl:text>_ts", NULL, def_ts);
	}

	void ui_module_unload(void) 
	{
		if (edp_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>) 
		{
			delete edp_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>;
		}
		fprintf(stderr, "module %s unloaded\n", __FILE__);
	}
}
</xsl:text>
</xsl:document>
<xsl:call-template name="irp6.edp.main.signals.h"/>
</xsl:template>

<!-- signals handling file .cc-->
<xsl:template name="irp6.edp.main.signals.h" match="*[substring(name(),1,4)='irp6']">
<xsl:variable name="name" select="name"/>
<xsl:document method="text" href="../edp_{$name}_rcsc_uimodule.h">



<xsl:text>
#ifndef __EDP_</xsl:text><xsl:value-of select="$name" /><xsl:text>_RCSC
#define __EDP_</xsl:text><xsl:value-of select="$name" /><xsl:text>_RCSC

#include &lt;iostream&gt;
#include &lt;vector&gt;

#include &lt;gtk/gtkbuilder.h&gt;
#include &lt;gtk/gtk.h&gt;

class edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>_rcsc
{
	public:

		edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>_rcsc(ui_config_entry &amp;entry);
		edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>_rcsc();
		~edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>_rcsc();


};

#endif /* __EDP_</xsl:text><xsl:value-of select="$name" /><xsl:text>_RCSC */
</xsl:text>

</xsl:document>
</xsl:template>

</xsl:stylesheet>
