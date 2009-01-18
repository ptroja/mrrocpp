<?xml version="1.0" encoding="UTF-8"?>
<!--
MRROC++ GUI generator
Axis_ts window callback signals
 -->
<xsl:stylesheet version="1.0" xmlns:xsl="http://www.w3.org/1999/XSL/Transform">
<xsl:output method="text"/>

<!-- signals handling file .cc-->
<xsl:template name="irp6.axis.ts.main.signals.cc" match="*[substring(name(),1,4)='irp6']">
<xsl:variable name="name" select="name"/>
<xsl:variable name="fullName" select="fullName"/>
<xsl:document method="text" href="../signals/{$name}_axis_ts_widget.cc">


<xsl:text>
#include &lt;iostream&gt;
#include &lt;gtk/gtk.h&gt;
#include &lt;glib.h&gt;
#include "ui_model.h"
#include "</xsl:text><xsl:value-of select="$name" /><xsl:text>_axis_ts_widget.h"


edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>_axis_ts::edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>_axis_ts(ui_widget_entry &amp;entry) 
{
}

static edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>_axis_ts *axis_ts_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>;


extern "C"
{
	void on_arrow_button_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_axis_ts (GtkButton* button, gpointer userdata)
	{
		std::cout &lt;&lt; "skopiuj wartosci dla </xsl:text><xsl:value-of select="$fullName" /><xsl:text> axis_ts" &lt;&lt; std::endl;
	}
	
	void on_read_button_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_axis_ts (GtkButton* button, gpointer user_data)
	{
		std::cout &lt;&lt; "wczytaj wartosci dla </xsl:text><xsl:value-of select="$fullName" /><xsl:text> axis_ts" &lt;&lt; std::endl;
	}
	
	void on_set_button_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_axis_ts (GtkButton* button, gpointer user_data)
	{
		std::cout &lt;&lt; "ustaw wartosci dla </xsl:text><xsl:value-of select="$fullName" /><xsl:text> axis_ts" &lt;&lt; std::endl;
	}
	
	
	void ui_widget_init(ui_widget_entry &amp;entry) 
	{
		axis_ts_</xsl:text><xsl:value-of select="$fullName" /><xsl:text> = new edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>_axis_ts(entry);
		fprintf(stderr, "widget %s loaded\n", __FILE__);
	}

	void ui_widget_unload(void) 
	{
		if (axis_ts_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>) 
		{
			delete axis_ts_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>;
		}
		fprintf(stderr, "widget %s unloaded\n", __FILE__);
	}
}
</xsl:text>
</xsl:document>
<xsl:call-template name="irp6.axis.ts.main.signals.h"/>
</xsl:template>

<!-- signals handling file .h-->
<xsl:template name="irp6.axis.ts.main.signals.h" match="*[substring(name(),1,4)='irp6']">
<xsl:variable name="name" select="name"/>
<xsl:document method="text" href="../signals/{$name}_axis_ts_widget.h">



<xsl:text>
#ifndef __EDP_</xsl:text><xsl:value-of select="$name" /><xsl:text>_AXIS_TS
#define __EDP_</xsl:text><xsl:value-of select="$name" /><xsl:text>_AXIS_TS

#include &lt;iostream&gt;
#include &lt;vector&gt;

#include &lt;gtk/gtkbuilder.h&gt;
#include &lt;gtk/gtk.h&gt;

class edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>_axis_ts
{
	public:

		edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>_axis_ts(ui_widget_entry &amp;entry);
		edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>_axis_ts();
		~edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>_axis_ts();


};

#endif /* __EDP_</xsl:text><xsl:value-of select="$name" /><xsl:text>_AXIS_TS */
</xsl:text>

</xsl:document>
</xsl:template>

</xsl:stylesheet>
