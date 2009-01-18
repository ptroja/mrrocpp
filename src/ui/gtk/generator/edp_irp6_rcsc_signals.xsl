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
<xsl:variable name="irp6EDPNumber" select="irp6EDPNumber"/>
<xsl:variable name="axis_xyz" select="axis_xyz"/>
<xsl:variable name="axis_ts" select="axis_ts"/>
<xsl:variable name="euler_xyz" select="euler_xyz"/>
<xsl:variable name="euler_ts" select="euler_ts"/>
<xsl:document method="text" href="../signals/edp_{$name}_widget.cc">



<xsl:text>
#include &lt;iostream&gt;
#include &lt;gtk/gtk.h&gt;
#include &lt;glib.h&gt;
#include "ui_model.h"
#include "edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>_widget.h"


edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>::edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>(ui_widget_entry &amp;entry)
{	
}

edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>::~edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>()
{
}

static edp_</xsl:text><xsl:value-of select="$name" /><xsl:text> *edp_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>;


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
		gboolean isFile = 0;
		gint choice;
		choice = gtk_combo_box_get_active (comboBox);

		switch (choice)
		{
		case 0: </xsl:text><xsl:if test="$irp6EDPNumber &gt;= 0"><xsl:text>std::cout &lt;&lt; "Servo algorithm window chosen" &lt;&lt; std::endl; ChosenFile = "</xsl:text><xsl:value-of select="$name" /><xsl:text>_servo_algorithm.xml"; isFile = 1;</xsl:text></xsl:if><xsl:text> break;
		case 1: </xsl:text><xsl:if test="$irp6EDPNumber &gt;= 0"><xsl:text>std::cout &lt;&lt; "Internal window chosen" &lt;&lt; std::endl; ChosenFile = "</xsl:text><xsl:value-of select="$name" /><xsl:text>_int.xml"; isFile = 1;</xsl:text></xsl:if><xsl:text> break;
		case 2: </xsl:text><xsl:if test="$irp6EDPNumber &gt;= 0"><xsl:text>std::cout &lt;&lt; "Increment window chosen" &lt;&lt; std::endl; ChosenFile = "</xsl:text><xsl:value-of select="$name" /><xsl:text>_inc.xml"; isFile = 1;</xsl:text></xsl:if><xsl:text> break;
		case 3: </xsl:text><xsl:if test="$euler_xyz &gt;= 0"><xsl:text>std::cout &lt;&lt; "XYZ Euler ZYZ window chosen" &lt;&lt; std::endl; ChosenFile = "</xsl:text><xsl:value-of select="$name" /><xsl:text>_euler_xyz.xml"; isFile = 1;</xsl:text></xsl:if><xsl:text> break;
		case 4: </xsl:text><xsl:if test="$axis_xyz &gt;= 0"><xsl:text>std::cout &lt;&lt; "XYZ Angle Axis window chosen" &lt;&lt; std::endl; ChosenFile = "</xsl:text><xsl:value-of select="$name" /><xsl:text>_axis_xyz.xml"; isFile = 1;</xsl:text></xsl:if><xsl:text> break;
		case 5: </xsl:text><xsl:if test="$axis_ts &gt;= 0"><xsl:text>std::cout &lt;&lt; "TS Angle Axis window chosen" &lt;&lt; std::endl; ChosenFile = "</xsl:text><xsl:value-of select="$name" /><xsl:text>_axis_ts.xml"; isFile = 1;</xsl:text></xsl:if><xsl:text> break;
		case 6: </xsl:text><xsl:if test="$euler_ts &gt;= 0"><xsl:text>std::cout &lt;&lt; "TS Euler ZYZ window chosen" &lt;&lt; std::endl; ChosenFile = "</xsl:text><xsl:value-of select="$name" /><xsl:text>_euler_ts.xml"; isFile = 1;</xsl:text></xsl:if><xsl:text> break;
		default: std::cout &lt;&lt; "Something is not working properly!" &lt;&lt; std::endl;
		}
		if (isFile)
		{
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
		
	}	

	void ui_widget_init(ui_widget_entry &amp;entry) 
	{
		edp_</xsl:text><xsl:value-of select="$fullName" /><xsl:text> = new edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>(entry);
		fprintf(stderr, "widget %s loaded\n", __FILE__);
		
		</xsl:text><xsl:if test="$irp6EDPNumber &gt;= 0"><xsl:text>const char * def_servo_algorithm = "</xsl:text><xsl:value-of select="$name" /><xsl:text>_servo_algorithm.xml"; new ui_widget_entry(def_servo_algorithm);</xsl:text></xsl:if><xsl:text>
		</xsl:text><xsl:if test="$irp6EDPNumber &gt;= 0"><xsl:text>const char * def_int = "</xsl:text><xsl:value-of select="$name" /><xsl:text>_int.xml"; new ui_widget_entry(def_int);</xsl:text></xsl:if><xsl:text>
		</xsl:text><xsl:if test="$irp6EDPNumber &gt;= 0"><xsl:text>const char * def_inc = "</xsl:text><xsl:value-of select="$name" /><xsl:text>_inc.xml"; new ui_widget_entry(def_inc);</xsl:text></xsl:if><xsl:text>
		</xsl:text><xsl:if test="$axis_xyz &gt;= 0"><xsl:text>const char * def_axis_xyz = "</xsl:text><xsl:value-of select="$name" /><xsl:text>_axis_xyz.xml"; new ui_widget_entry(def_axis_xyz);</xsl:text></xsl:if><xsl:text>
		</xsl:text><xsl:if test="$euler_xyz &gt;= 0"><xsl:text>const char * def_euler_xyz = "</xsl:text><xsl:value-of select="$name" /><xsl:text>_euler_xyz.xml"; new ui_widget_entry(def_euler_xyz);</xsl:text></xsl:if><xsl:text>
		</xsl:text><xsl:if test="$axis_ts &gt;= 0"><xsl:text>const char * def_axis_ts = "</xsl:text><xsl:value-of select="$name" /><xsl:text>_axis_ts.xml"; new ui_widget_entry(def_axis_ts);</xsl:text></xsl:if><xsl:text>
		</xsl:text><xsl:if test="$euler_ts &gt;= 0"><xsl:text>const char * def_euler_ts = "</xsl:text><xsl:value-of select="$name" /><xsl:text>_euler_ts.xml"; new ui_widget_entry(def_euler_ts);</xsl:text></xsl:if><xsl:text>
	}

	void ui_widget_unload(void) 
	{
		if (edp_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>) 
		{
			delete edp_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>;
		}
		fprintf(stderr, "widget %s unloaded\n", __FILE__);
	}
}
</xsl:text>
</xsl:document>
<xsl:call-template name="irp6.edp.main.signals.h"/>
</xsl:template>

<!-- signals handling file .cc-->
<xsl:template name="irp6.edp.main.signals.h" match="*[substring(name(),1,4)='irp6']">
<xsl:variable name="name" select="name"/>
<xsl:document method="text" href="../signals/edp_{$name}_widget.h">



<xsl:text>
#ifndef __EDP_</xsl:text><xsl:value-of select="$name" /><xsl:text>
#define __EDP_</xsl:text><xsl:value-of select="$name" /><xsl:text>

#include &lt;iostream&gt;
#include &lt;vector&gt;

#include &lt;gtk/gtkbuilder.h&gt;
#include &lt;gtk/gtk.h&gt;

class edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>
{
	public:

		edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>(ui_widget_entry &amp;entry);
		edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>();
		~edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>();


};

#endif /* __EDP_</xsl:text><xsl:value-of select="$name" /><xsl:text> */
</xsl:text>

</xsl:document>
</xsl:template>

</xsl:stylesheet>
