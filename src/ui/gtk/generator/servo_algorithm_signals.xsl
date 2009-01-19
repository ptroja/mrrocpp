<?xml version="1.0" encoding="UTF-8"?>
<!--
MRROC++ GUI generator
Servo_algorithm window callback signals
 -->
<xsl:stylesheet version="1.0" xmlns:xsl="http://www.w3.org/1999/XSL/Transform">
<xsl:output method="text"/>

<!-- signals handling file .cc-->
<xsl:template name="irp6.servo.main.signals.cc" match="*[substring(name(),1,4)='irp6']">
<xsl:variable name="name" select="name"/>
<xsl:variable name="fullName" select="fullName"/>
<xsl:variable name="irp6EDPNumber" select="irp6EDPNumber"/>
<xsl:document method="text" href="../signals/{$name}_servo_algorithm_widget.cc">


<xsl:text>
#include &lt;iostream&gt;
#include &lt;gtk/gtk.h&gt;
#include &lt;glib.h&gt;
#include "ui_model.h"
#include "</xsl:text><xsl:value-of select="$name" /><xsl:text>_servo_algorithm_widget.h"


edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>_servo_algorithm::edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>_servo_algorithm(ui_widget_entry &amp;entry) 
{
}

static edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>_servo_algorithm *servo_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>;


extern "C"
{
	void on_arrow_button_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_servo (GtkButton* button, gpointer userdata)
	{
		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder &amp; thisBuilder = ((*ChoseEntry).getBuilder());
        
		</xsl:text><xsl:call-template name="irp6.servo.repeat.signals.cc">
    		<xsl:with-param name="irp6EDPNumber" select="$irp6EDPNumber"/>
			<xsl:with-param name="i" select="1"/>
 		</xsl:call-template><xsl:text>
	}
	
	void on_read_button_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_servo (GtkButton* button, gpointer user_data)
	{
		std::cout &lt;&lt; "wczytaj wartosci dla </xsl:text><xsl:value-of select="$fullName" /><xsl:text> servo" &lt;&lt; std::endl;
	}
	
	void on_set_button_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_servo (GtkButton* button, gpointer user_data)
	{
		std::cout &lt;&lt; "ustaw wartosci dla </xsl:text><xsl:value-of select="$fullName" /><xsl:text> servo" &lt;&lt; std::endl;
	}
	
	
	void ui_widget_init(ui_widget_entry &amp;entry) 
	{
		servo_</xsl:text><xsl:value-of select="$fullName" /><xsl:text> = new edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>_servo_algorithm(entry);
		fprintf(stderr, "widget %s loaded\n", __FILE__);
	}

	void ui_widget_unload(void) 
	{
		if (servo_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>) 
		{
			delete servo_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>;
		}
		fprintf(stderr, "widget %s unloaded\n", __FILE__);
	}
}
</xsl:text>
</xsl:document>


<xsl:call-template name="irp6.servo.main.signals.h"/>
</xsl:template>

<!-- irp6 servo algorithm repeatable part -->
<xsl:template name="irp6.servo.repeat.signals.cc">
<xsl:param name="irp6EDPNumber"/>
<xsl:param name="i"/>
	<xsl:if test="$i &lt;= $irp6EDPNumber*2">
	<xsl:text>
        GtkEntry * entry</xsl:text><xsl:value-of select="$i" /><xsl:text> = GTK_ENTRY(gtk_builder_get_object(&amp;thisBuilder, "entry</xsl:text><xsl:value-of select="$i" /><xsl:text>"));
        GtkSpinButton * spin</xsl:text><xsl:value-of select="$i" /><xsl:text> = GTK_SPIN_BUTTON(gtk_builder_get_object(&amp;thisBuilder, "spinbutton</xsl:text><xsl:value-of select="$i" /><xsl:text>"));
        gtk_spin_button_set_value(spin</xsl:text><xsl:value-of select="$i" /><xsl:text>, atof(gtk_entry_get_text(entry</xsl:text><xsl:value-of select="$i" /><xsl:text>)));
	</xsl:text>
       </xsl:if>
	<!-- for loop --> 
       <xsl:if test="$i &lt;= $irp6EDPNumber*2">
          <xsl:call-template name="irp6.servo.repeat.signals.cc">
              <xsl:with-param name="i">
                  <xsl:value-of select="$i + 1"/>
              </xsl:with-param>
              <xsl:with-param name="irp6EDPNumber">
                  <xsl:value-of select="$irp6EDPNumber"/>
              </xsl:with-param>
          </xsl:call-template>
       </xsl:if>
</xsl:template>

<!-- signals handling file .h-->
<xsl:template name="irp6.servo.main.signals.h" match="*[substring(name(),1,4)='irp6']">
<xsl:variable name="name" select="name"/>
<xsl:document method="text" href="../signals/{$name}_servo_algorithm_widget.h">



<xsl:text>
#ifndef __EDP_</xsl:text><xsl:value-of select="$name" /><xsl:text>_SERVO_ALGORITHM
#define __EDP_</xsl:text><xsl:value-of select="$name" /><xsl:text>_SERVO_ALGORITHM

#include &lt;iostream&gt;
#include &lt;vector&gt;

#include &lt;gtk/gtkbuilder.h&gt;
#include &lt;gtk/gtk.h&gt;

class edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>_servo_algorithm
{
	public:

		edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>_servo_algorithm(ui_widget_entry &amp;entry);
		~edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>_servo_algorithm();


};

#endif /* __EDP_</xsl:text><xsl:value-of select="$name" /><xsl:text>_SERVO_ALGORITHM */
</xsl:text>

</xsl:document>
</xsl:template>

</xsl:stylesheet>
