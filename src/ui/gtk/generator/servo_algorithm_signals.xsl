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
<xsl:document method="text" href="../{$name}_servo_algorithm_uimodule.cc">


<xsl:text>
#include &lt;iostream&gt;
#include &lt;gtk/gtk.h&gt;
#include &lt;glib.h&gt;
#include "ui_model.h"
#include "</xsl:text><xsl:value-of select="$name" /><xsl:text>_servo_algorithm_uimodule.h"


edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>_servo_algorithm::edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>_servo_algorithm(ui_config_entry &amp;entry) 
{
}

static edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>_servo_algorithm *servo_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>;


extern "C"
{
	void on_arrow_button_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_servo (GtkButton* button, gpointer userdata)
	{
		std::cout &lt;&lt; "skopiuj wartosci dla </xsl:text><xsl:value-of select="$fullName" /><xsl:text> servo" &lt;&lt; std::endl;
	}
	
	void on_read_button_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_servo (GtkButton* button, gpointer user_data)
	{
		std::cout &lt;&lt; "wczytaj wartosci dla </xsl:text><xsl:value-of select="$fullName" /><xsl:text> servo" &lt;&lt; std::endl;
	}
	
	void on_set_button_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_servo (GtkButton* button, gpointer user_data)
	{
		std::cout &lt;&lt; "ustaw wartosci dla </xsl:text><xsl:value-of select="$fullName" /><xsl:text> servo" &lt;&lt; std::endl;
	}
	
	
	void ui_module_init(ui_config_entry &amp;entry) 
	{
		servo_</xsl:text><xsl:value-of select="$fullName" /><xsl:text> = new edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>_servo_algorithm(entry);
		fprintf(stderr, "module %s loaded\n", __FILE__);
	}

	void ui_module_unload(void) 
	{
		if (servo_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>) 
		{
			delete servo_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>;
		}
		fprintf(stderr, "module %s unloaded\n", __FILE__);
	}
}
</xsl:text>
</xsl:document>
<xsl:call-template name="irp6.servo.main.signals.h"/>
</xsl:template>

<!-- signals handling file .h-->
<xsl:template name="irp6.servo.main.signals.h" match="*[substring(name(),1,4)='irp6']">
<xsl:variable name="name" select="name"/>
<xsl:document method="text" href="../{$name}_servo_algorithm_uimodule.h">



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

		edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>_servo_algorithm(ui_config_entry &amp;entry);
		edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>_servo_algorithm();
		~edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>_servo_algorithm();


};

#endif /* __EDP_</xsl:text><xsl:value-of select="$name" /><xsl:text>_SERVO_ALGORITHM */
</xsl:text>

</xsl:document>
</xsl:template>

</xsl:stylesheet>
