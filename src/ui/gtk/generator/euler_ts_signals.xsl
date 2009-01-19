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
<xsl:variable name="euler_ts" select="euler_ts"/>
<xsl:document method="text" href="../signals/{$name}_euler_ts_widget.cc">


<xsl:text>
#include &lt;iostream&gt;
#include &lt;gtk/gtk.h&gt;
#include &lt;glib.h&gt;
#include "ui_model.h"
#include "</xsl:text><xsl:value-of select="$name" /><xsl:text>_euler_ts_widget.h"


edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>_euler_ts::edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>_euler_ts(ui_widget_entry &amp;entry) 
{
}

static edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>_euler_ts *euler_ts_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>;


extern "C"
{
	void on_arrow_button_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_euler_ts (GtkButton* button, gpointer userdata)
	{
		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder &amp; thisBuilder = ((*ChoseEntry).getBuilder());
        
		</xsl:text><xsl:call-template name="irp6.euler.ts.repeat.signals.cc">
    		<xsl:with-param name="euler_ts" select="$euler_ts"/>
			<xsl:with-param name="i" select="1"/>
 		</xsl:call-template><xsl:text>	
 	}
	
	void on_read_button_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_euler_ts (GtkButton* button, gpointer user_data)
	{
		std::cout &lt;&lt; "wczytaj wartosci dla </xsl:text><xsl:value-of select="$fullName" /><xsl:text> euler_ts" &lt;&lt; std::endl;
	}
	
	void on_set_button_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_euler_ts (GtkButton* button, gpointer user_data)
	{
		std::cout &lt;&lt; "ustaw wartosci dla </xsl:text><xsl:value-of select="$fullName" /><xsl:text> euler_ts" &lt;&lt; std::endl;
	}
	
	
	void ui_widget_init(ui_widget_entry &amp;entry) 
	{
		euler_ts_</xsl:text><xsl:value-of select="$fullName" /><xsl:text> = new edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>_euler_ts(entry);
		fprintf(stderr, "widget %s loaded\n", __FILE__);
	}

	void ui_widget_unload(void) 
	{
		if (euler_ts_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>) 
		{
			delete euler_ts_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>;
		}
		fprintf(stderr, "widget %s unloaded\n", __FILE__);
	}
}
</xsl:text>
</xsl:document>
<xsl:call-template name="irp6.euler.ts.main.signals.h"/>
</xsl:template>

<!-- irp6 axis ts handling signals .cc repeatable part -->
<xsl:template name="irp6.euler.ts.repeat.signals.cc">
<xsl:param name="euler_ts"/>
<xsl:param name="i"/>
	<xsl:if test="$i &lt;= $euler_ts">
	<xsl:text>
        GtkEntry * entry</xsl:text><xsl:value-of select="$i" /><xsl:text> = GTK_ENTRY(gtk_builder_get_object(&amp;thisBuilder, "entry</xsl:text><xsl:value-of select="$i" /><xsl:text>"));
        GtkSpinButton * spin</xsl:text><xsl:value-of select="$i" /><xsl:text> = GTK_SPIN_BUTTON(gtk_builder_get_object(&amp;thisBuilder, "spinbutton</xsl:text><xsl:value-of select="$i" /><xsl:text>"));
        gtk_spin_button_set_value(spin</xsl:text><xsl:value-of select="$i" /><xsl:text>, atof(gtk_entry_get_text(entry</xsl:text><xsl:value-of select="$i" /><xsl:text>)));
	</xsl:text>
       </xsl:if>
	<!-- for loop --> 
       <xsl:if test="$i &lt;= $euler_ts">
          <xsl:call-template name="irp6.euler.ts.repeat.signals.cc">
              <xsl:with-param name="i">
                  <xsl:value-of select="$i + 1"/>
              </xsl:with-param>
              <xsl:with-param name="euler_ts">
                  <xsl:value-of select="$euler_ts"/>
              </xsl:with-param>
          </xsl:call-template>
       </xsl:if>
</xsl:template>

<!-- signals handling file .h-->
<xsl:template name="irp6.euler.ts.main.signals.h" match="*[substring(name(),1,4)='irp6']">
<xsl:variable name="name" select="name"/>
<xsl:document method="text" href="../signals/{$name}_euler_ts_widget.h">



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

		edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>_euler_ts(ui_widget_entry &amp;entry);
		~edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>_euler_ts();


};

#endif /* __EDP_</xsl:text><xsl:value-of select="$name" /><xsl:text>_EULER_TS */
</xsl:text>

</xsl:document>
</xsl:template>

</xsl:stylesheet>
