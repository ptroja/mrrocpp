<?xml version="1.0" encoding="UTF-8"?>
<!--
MRROC++ GUI generator
Festival window callback signals
 -->
<xsl:stylesheet version="1.0" xmlns:xsl="http://www.w3.org/1999/XSL/Transform">

<!-- main festival window -->
<xsl:template name="festival.main.signals.cc" match="festival">
<xsl:variable name="name" select="name"/>
<xsl:variable name="fullName" select="fullName"/>
<xsl:document method="text" doctype-system="glade-2.0.dtd" indent="yes" version="1.0" href="../signals/{$name}_uimodule.cc">



#include &lt;iostream&gt;
#include &lt;gtk/gtk.h&gt;
#include &lt;glib.h&gt;
#include "ui_model.h"
#include "<xsl:value-of select="$name" />_uimodule.h"


<xsl:value-of select="$name" />::<xsl:value-of select="$name" />(ui_config_entry &amp;entry) 
{
}

static <xsl:value-of select="$name" /> *<xsl:value-of select="$fullName" />Panel;


extern "C"
{
	void on_say_button_clicked_<xsl:value-of select="$fullName" /> (GtkButton* button, gpointer userdata)
	{
		std::cout &lt;&lt; "Wcisniety przycisk Say dla <xsl:value-of select="$fullName" />" &lt;&lt; std::endl;
	}
	
	void ui_module_init(ui_config_entry &amp;entry) 
	{
		<xsl:value-of select="$fullName" />Panel = new <xsl:value-of select="$name" />(entry);
		fprintf(stderr, "module %s loaded\n", __FILE__);
	}

	void ui_module_unload(void) 
	{
		if (<xsl:value-of select="$fullName" />Panel) 
		{
			delete <xsl:value-of select="$fullName" />Panel;
		}
		fprintf(stderr, "module %s unloaded\n", __FILE__);
	}
}


</xsl:document>	
<xsl:call-template name="festival.main.signals.h"/>
</xsl:template>

<!-- signals handling file .h-->
<xsl:template name="festival.main.signals.h">
<xsl:variable name="name" select="name"/>
<xsl:document method="text" href="../signals/{$name}_uimodule.h">


#ifndef __<xsl:value-of select="$name" />
#define __<xsl:value-of select="$name" />

#include &lt;iostream&gt;
#include &lt;vector&gt;

#include &lt;gtk/gtkbuilder.h&gt;
#include &lt;gtk/gtk.h&gt;

class <xsl:value-of select="$name" />
{
	public:

		<xsl:value-of select="$name" />(ui_config_entry &amp;entry);
		~<xsl:value-of select="$name" />();


};

#endif /* __<xsl:value-of select="$name" /> */

</xsl:document>
</xsl:template>

</xsl:stylesheet>
