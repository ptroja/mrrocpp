<?xml version="1.0" encoding="UTF-8"?>
<!--
MRROC++ GUI generator
Festival window callback signals
 -->
<xsl:stylesheet version="1.0" xmlns:xsl="http://www.w3.org/1999/XSL/Transform">
<xsl:output method="text"/>

<!-- signals handling file .cc-->
<xsl:template name="festival.main.signals.cc">
<xsl:variable name="name" select="name"/>
<xsl:document method="text" href="../signals/speaker_uimodule.cc">


<xsl:text>
#include &lt;iostream&gt;
#include &lt;gtk/gtk.h&gt;
#include &lt;glib.h&gt;
#include "ui_model.h"
#include "festival_uimodule.h"


festival::festival(ui_config_entry &amp;entry) 
{
}

static festival *festivalPanel;


extern "C"
{
	void on_say_button_clicked (GtkButton* button, gpointer userdata)
	{
		std::cout &lt;&lt; "Say button clicked in festival window" &lt;&lt; std::endl;
	}	
	
	void ui_module_init(ui_config_entry &amp;entry) 
	{
		festivalPanel = new festival(entry);
		fprintf(stderr, "module %s loaded\n", __FILE__);
	}

	void ui_module_unload(void) 
	{
		if (festivalPanel) 
		{
			delete festivalPanel;
		}
		fprintf(stderr, "module %s unloaded\n", __FILE__);
	}
}
</xsl:text>
</xsl:document>
<xsl:call-template name="festival.main.signals.h"/>
</xsl:template>


<!-- signals handling file .h-->
<xsl:template name="festival.main.signals.h" match="festival">
<xsl:variable name="name" select="name"/>
<xsl:document method="text" href="../signals/festival_uimodule.h">

<xsl:text>
#ifndef __festival
#define __festival

#include &lt;iostream&gt;
#include &lt;vector&gt;

#include &lt;gtk/gtkbuilder.h&gt;
#include &lt;gtk/gtk.h&gt;

class festival
{
	public:

		festival(ui_config_entry &amp;entry);
		festival();
		~festival();


};

#endif /* __festival */
</xsl:text>
</xsl:document>
</xsl:template>

</xsl:stylesheet>
