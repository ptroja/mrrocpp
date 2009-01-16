<?xml version="1.0" encoding="UTF-8"?>
<!--
MRROC++ GUI generator
Euler_ts window callback signals
 -->
<xsl:stylesheet version="1.0" xmlns:xsl="http://www.w3.org/1999/XSL/Transform">
<xsl:output method="text"/>

<!-- signals handling file .cc-->
<xsl:template name="irp6.euler.ts.main.signals.cc" match="*[substring(name(),1,4)='irp6']">
<xsl:variable name="name" select="name"/>
<xsl:variable name="fullName" select="fullName"/>
<xsl:document method="text" href="../{$name}_euler_ts_uimodule.cc">


<xsl:text>
#include &lt;iostream&gt;
#include &lt;gtk/gtk.h&gt;
#include &lt;glib.h&gt;
#include "ui_model.h"
#include "</xsl:text><xsl:value-of select="$name" /><xsl:text>_euler_ts_uimodule.h"


edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>_euler_ts::edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>_euler_ts(ui_config_entry &amp;entry) 
{
}

static edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>_euler_ts *euler_ts_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>;


extern "C"
{
	void on_arrow_button_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_euler_ts (GtkButton* button, gpointer userdata)
	{
		std::cout &lt;&lt; "skopiuj wartosci dla </xsl:text><xsl:value-of select="$fullName" /><xsl:text> euler_ts" &lt;&lt; std::endl;
	}
	
	void on_read_button_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_euler_ts (GtkButton* button, gpointer user_data)
	{
		std::cout &lt;&lt; "wczytaj wartosci dla </xsl:text><xsl:value-of select="$fullName" /><xsl:text> euler_ts" &lt;&lt; std::endl;
	}
	
	void on_set_button_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_euler_ts (GtkButton* button, gpointer user_data)
	{
		std::cout &lt;&lt; "ustaw wartosci dla </xsl:text><xsl:value-of select="$fullName" /><xsl:text> euler_ts" &lt;&lt; std::endl;
	}
	
	
	void ui_module_init(ui_config_entry &amp;entry) 
	{
		euler_ts_</xsl:text><xsl:value-of select="$fullName" /><xsl:text> = new edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>_euler_ts(entry);
		fprintf(stderr, "module %s loaded\n", __FILE__);
	}

	void ui_module_unload(void) 
	{
		if (euler_ts_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>) 
		{
			delete euler_ts_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>;
		}
		fprintf(stderr, "module %s unloaded\n", __FILE__);
	}
}
</xsl:text>
</xsl:document>
<xsl:call-template name="irp6.euler.ts.main.signals.h"/>
</xsl:template>

<!-- signals handling file .h-->
<xsl:template name="irp6.euler.ts.main.signals.h" match="*[substring(name(),1,4)='irp6']">
<xsl:variable name="name" select="name"/>
<xsl:document method="text" href="../{$name}_euler_ts_uimodule.h">



<xsl:text>
#ifndef __EDP_</xsl:text><xsl:value-of select="$name" /><xsl:text>_EULER_TS
#define __EDP_</xsl:text><xsl:value-of select="$name" /><xsl:text>_EULER_TS

#include &lt;iostream&gt;
#include &lt;vector&gt;

#include &lt;gtk/gtkbuilder.h&gt;
#include &lt;gtk/gtk.h&gt;

class edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>_euler_ts
{
	public:

		edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>_euler_ts(ui_config_entry &amp;entry);
		edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>_euler_ts();
		~edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>_euler_ts();


};

#endif /* __EDP_</xsl:text><xsl:value-of select="$name" /><xsl:text>_EULER_TS */
</xsl:text>

</xsl:document>
</xsl:template>

</xsl:stylesheet>
