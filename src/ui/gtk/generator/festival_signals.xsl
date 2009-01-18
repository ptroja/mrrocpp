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
<xsl:document method="text" doctype-system="glade-2.0.dtd" indent="yes" version="1.0" href="../signals/{$name}_widget.cc">


<xsl:text>
#include &lt;iostream&gt;
#include &lt;gtk/gtk.h&gt;
#include &lt;glib.h&gt;
#include "ui_model.h"
#include "</xsl:text><xsl:value-of select="$name" /><xsl:text>_widget.h"


</xsl:text><xsl:value-of select="$name" /><xsl:text>::</xsl:text><xsl:value-of select="$name" /><xsl:text>(ui_widget_entry &amp;entry) 
{
}

static </xsl:text><xsl:value-of select="$name" /><xsl:text> *</xsl:text><xsl:value-of select="$fullName" /><xsl:text>Panel;


extern "C"
{
	void on_say_button_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text> (GtkButton* button, gpointer userdata)
	{
		std::cout &lt;&lt; "Wcisniety przycisk Say dla </xsl:text><xsl:value-of select="$fullName" /><xsl:text>" &lt;&lt; std::endl;
	}
	
	void ui_widget_init(ui_widget_entry &amp;entry) 
	{
		</xsl:text><xsl:value-of select="$fullName" /><xsl:text>Panel = new </xsl:text><xsl:value-of select="$name" /><xsl:text>(entry);
		fprintf(stderr, "widget %s loaded\n", __FILE__);
	}

	void ui_widget_unload(void) 
	{
		if (</xsl:text><xsl:value-of select="$fullName" /><xsl:text>Panel) 
		{
			delete </xsl:text><xsl:value-of select="$fullName" /><xsl:text>Panel;
		}
		fprintf(stderr, "widget %s unloaded\n", __FILE__);
	}
}
</xsl:text>

</xsl:document>	
<xsl:call-template name="festival.main.signals.h"/>
</xsl:template>

<!-- signals handling file .h-->
<xsl:template name="festival.main.signals.h">
<xsl:variable name="name" select="name"/>
<xsl:document method="text" href="../signals/{$name}_widget.h">

<xsl:text>
#ifndef __</xsl:text><xsl:value-of select="$name" /><xsl:text>
#define __</xsl:text><xsl:value-of select="$name" /><xsl:text>

#include &lt;iostream&gt;
#include &lt;vector&gt;

#include &lt;gtk/gtkbuilder.h&gt;
#include &lt;gtk/gtk.h&gt;

class </xsl:text><xsl:value-of select="$name" /><xsl:text>
{
	public:

		</xsl:text><xsl:value-of select="$name" /><xsl:text>(ui_widget_entry &amp;entry);
		</xsl:text><xsl:value-of select="$name" /><xsl:text>();
		~</xsl:text><xsl:value-of select="$name" /><xsl:text>();


};

#endif /* __</xsl:text><xsl:value-of select="$name" /><xsl:text> */
</xsl:text>
</xsl:document>
</xsl:template>

</xsl:stylesheet>
