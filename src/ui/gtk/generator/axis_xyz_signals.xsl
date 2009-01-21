<?xml version="1.0" encoding="UTF-8"?>
<!--
MRROC++ GUI generator
Axis XYZ window callback signals
 -->
<xsl:stylesheet version="1.0" xmlns:xsl="http://www.w3.org/1999/XSL/Transform">
<xsl:output method="text"/>

<!-- signals handling file .cc-->
<xsl:template name="irp6.axis.xyz.main.signals.cc" match="*[substring(name(),1,4)='irp6']">
<xsl:variable name="name" select="name"/>
<xsl:variable name="fullName" select="fullName"/>
<xsl:variable name="irp6EDPNumber" select="axis_xyz"/>
<xsl:document method="text" href="../signals/{$name}_axis_xyz_widget.cc">


<xsl:text>
#include &lt;iostream&gt;
#include &lt;gtk/gtk.h&gt;
#include &lt;glib.h&gt;
#include "ui_model.h"
#include "</xsl:text><xsl:value-of select="$name" /><xsl:text>_axis_xyz_widget.h"

double wl;
double l_eps = 0;

edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>_axis_xyz::edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>_axis_xyz(ui_widget_entry &amp;entry) 
{
}

static edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>_axis_xyz *axis_xyz_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>;


extern "C"
{
	void on_arrow_button_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_axis_xyz (GtkButton* button, gpointer userdata)
	{
		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder &amp; thisBuilder = ((*ChoseEntry).getBuilder());
        
		</xsl:text><xsl:call-template name="irp6.axis.xyz.repeat.signals.cc.1">
    		<xsl:with-param name="axis_xyz" select="$irp6EDPNumber"/>
			<xsl:with-param name="i" select="1"/>
 		</xsl:call-template><xsl:text>
	}
	
	void on_read_button_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_axis_xyz (GtkButton* button, gpointer user_data)
	{
		std::cout &lt;&lt; "wczytaj wartosci dla </xsl:text><xsl:value-of select="$fullName" /><xsl:text> axis_xyz" &lt;&lt; std::endl;
	}
	
	void on_execute_button_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_axis_xyz (GtkButton* button, gpointer user_data)
	{
		std::cout &lt;&lt; "Execute move dla </xsl:text><xsl:value-of select="$fullName" /><xsl:text> axis_xyz" &lt;&lt; std::endl;
	}
	
	void on_export_button_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_axis_xyz (GtkButton* button, gpointer user_data)
	{
		std::cout &lt;&lt; "Export dla </xsl:text><xsl:value-of select="$fullName" /><xsl:text> axis_xyz" &lt;&lt; std::endl;
	}
	
	void on_import_button_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_axis_xyz (GtkButton* button, gpointer user_data)
	{
		std::cout &lt;&lt; "Import dla </xsl:text><xsl:value-of select="$fullName" /><xsl:text> axis_xyz" &lt;&lt; std::endl;
	}
	
	
	void ui_widget_init(ui_widget_entry &amp;entry) 
	{
		axis_xyz_</xsl:text><xsl:value-of select="$fullName" /><xsl:text> = new edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>_axis_xyz(entry);
		fprintf(stderr, "widget %s loaded\n", __FILE__);
	}

	void ui_widget_unload(void) 
	{
		if (axis_xyz_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>) 
		{
			delete axis_xyz_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>;
		}
		fprintf(stderr, "widget %s unloaded\n", __FILE__);
	}
	
</xsl:text>
		<xsl:call-template name="for.each.edp.irp6.axis.xyz.signals.cc">
    			<xsl:with-param name="irp6EDPNumber" select="$irp6EDPNumber"/>
    			<xsl:with-param name="fullName" select="$fullName"/>
			<xsl:with-param name="i" select="1"/>
 		</xsl:call-template>
<xsl:text>
}
</xsl:text>
</xsl:document>
<xsl:call-template name="irp6.axis.xyz.main.signals.h"/>
</xsl:template>

<!-- irp6 axis xyz handling signals .cc repeatable part -->
<xsl:template name="irp6.axis.xyz.repeat.signals.cc.1">
<xsl:param name="axis_xyz"/>
<xsl:param name="i"/>
	<xsl:if test="$i &lt;= $axis_xyz">
	<xsl:text>
        GtkEntry * entry</xsl:text><xsl:value-of select="$i" /><xsl:text> = GTK_ENTRY(gtk_builder_get_object(&amp;thisBuilder, "entry</xsl:text><xsl:value-of select="$i" /><xsl:text>"));
        GtkSpinButton * spin</xsl:text><xsl:value-of select="$i" /><xsl:text> = GTK_SPIN_BUTTON(gtk_builder_get_object(&amp;thisBuilder, "spinbutton</xsl:text><xsl:value-of select="$i" /><xsl:text>"));
        gtk_spin_button_set_value(spin</xsl:text><xsl:value-of select="$i" /><xsl:text>, atof(gtk_entry_get_text(entry</xsl:text><xsl:value-of select="$i" /><xsl:text>)));
	</xsl:text>
       </xsl:if>
	<!-- for loop --> 
       <xsl:if test="$i &lt;= $axis_xyz">
          <xsl:call-template name="irp6.axis.xyz.repeat.signals.cc.1">
              <xsl:with-param name="i">
                  <xsl:value-of select="$i + 1"/>
              </xsl:with-param>
              <xsl:with-param name="axis_xyz">
                  <xsl:value-of select="$axis_xyz"/>
              </xsl:with-param>
          </xsl:call-template>
       </xsl:if>
</xsl:template>

<!-- handling signals .cc repeatable part -->
<xsl:template name="for.each.edp.irp6.axis.xyz.signals.cc">
<xsl:param name="irp6EDPNumber"/>
<xsl:param name="fullName"/>
<xsl:param name="i"/>
	<xsl:if test="$i &lt;= $irp6EDPNumber">
	<xsl:choose>
		<xsl:when test="$i &lt;= 3">
<xsl:text>
	void on_button</xsl:text><xsl:value-of select="($i*2)-1" /><xsl:text>_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_axis_xyz (GtkButton* button, gpointer userdata)
	{
 		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder &amp; thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1 = GTK_SPIN_BUTTON(gtk_builder_get_object(&amp;thisBuilder, "spinbuttonDown1"));
        GtkSpinButton * spin</xsl:text><xsl:value-of select="$i" /><xsl:text> = GTK_SPIN_BUTTON(gtk_builder_get_object(&amp;thisBuilder, "spinbutton</xsl:text><xsl:value-of select="$i" /><xsl:text>"));
        gtk_spin_button_set_value(spin</xsl:text><xsl:value-of select="$i" /><xsl:text>, gtk_spin_button_get_value(spin</xsl:text><xsl:value-of select="$i" /><xsl:text>) - gtk_spin_button_get_value(spinbuttonDown1));
        
        GtkSpinButton * spin4 = GTK_SPIN_BUTTON(gtk_builder_get_object(&amp;thisBuilder, "spinbutton4"));
        GtkSpinButton * spin5 = GTK_SPIN_BUTTON(gtk_builder_get_object(&amp;thisBuilder, "spinbutton5"));
        GtkSpinButton * spin6 = GTK_SPIN_BUTTON(gtk_builder_get_object(&amp;thisBuilder, "spinbutton6"));
        
        wl = sqrt(gtk_spin_button_get_value(spin4)*gtk_spin_button_get_value(spin4) + gtk_spin_button_get_value(spin5)*gtk_spin_button_get_value(spin5) + gtk_spin_button_get_value(spin6)*gtk_spin_button_get_value(spin6));
		if((wl &gt; 1 + l_eps) || (wl &lt; 1 - l_eps))
		{
			gtk_spin_button_set_value(spin4, gtk_spin_button_get_value(spin4) / wl);
			gtk_spin_button_set_value(spin5, gtk_spin_button_get_value(spin5) / wl);
			gtk_spin_button_set_value(spin6, gtk_spin_button_get_value(spin6) / wl);
		}
 	}
	
	void on_button</xsl:text><xsl:value-of select="($i*2)" /><xsl:text>_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_axis_xyz (GtkButton* button, gpointer userdata)
	{
 		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder &amp; thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1 = GTK_SPIN_BUTTON(gtk_builder_get_object(&amp;thisBuilder, "spinbuttonDown1"));
        GtkSpinButton * spin</xsl:text><xsl:value-of select="$i" /><xsl:text> = GTK_SPIN_BUTTON(gtk_builder_get_object(&amp;thisBuilder, "spinbutton</xsl:text><xsl:value-of select="$i" /><xsl:text>"));
        gtk_spin_button_set_value(spin</xsl:text><xsl:value-of select="$i" /><xsl:text>, gtk_spin_button_get_value(spin</xsl:text><xsl:value-of select="$i" /><xsl:text>) + gtk_spin_button_get_value(spinbuttonDown1));
        
        GtkSpinButton * spin4 = GTK_SPIN_BUTTON(gtk_builder_get_object(&amp;thisBuilder, "spinbutton4"));
        GtkSpinButton * spin5 = GTK_SPIN_BUTTON(gtk_builder_get_object(&amp;thisBuilder, "spinbutton5"));
        GtkSpinButton * spin6 = GTK_SPIN_BUTTON(gtk_builder_get_object(&amp;thisBuilder, "spinbutton6"));
        
        wl = sqrt(gtk_spin_button_get_value(spin4)*gtk_spin_button_get_value(spin4) + gtk_spin_button_get_value(spin5)*gtk_spin_button_get_value(spin5) + gtk_spin_button_get_value(spin6)*gtk_spin_button_get_value(spin6));
		if((wl &gt; 1 + l_eps) || (wl &lt; 1 - l_eps))
		{
			gtk_spin_button_set_value(spin4, gtk_spin_button_get_value(spin4) / wl);
			gtk_spin_button_set_value(spin5, gtk_spin_button_get_value(spin5) / wl);
			gtk_spin_button_set_value(spin6, gtk_spin_button_get_value(spin6) / wl);
		}
 	}   
</xsl:text>
 		</xsl:when>
 		<xsl:when test="$i &gt;= 7">
<xsl:text>
	void on_button</xsl:text><xsl:value-of select="($i*2)-1" /><xsl:text>_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_axis_xyz (GtkButton* button, gpointer userdata)
	{
 		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder &amp; thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1 = GTK_SPIN_BUTTON(gtk_builder_get_object(&amp;thisBuilder, "spinbuttonDown1"));
        GtkSpinButton * spin</xsl:text><xsl:value-of select="$i" /><xsl:text> = GTK_SPIN_BUTTON(gtk_builder_get_object(&amp;thisBuilder, "spinbutton</xsl:text><xsl:value-of select="$i" /><xsl:text>"));
        gtk_spin_button_set_value(spin</xsl:text><xsl:value-of select="$i" /><xsl:text>, gtk_spin_button_get_value(spin</xsl:text><xsl:value-of select="$i" /><xsl:text>) - gtk_spin_button_get_value(spinbuttonDown1));
        
        GtkSpinButton * spin4 = GTK_SPIN_BUTTON(gtk_builder_get_object(&amp;thisBuilder, "spinbutton4"));
        GtkSpinButton * spin5 = GTK_SPIN_BUTTON(gtk_builder_get_object(&amp;thisBuilder, "spinbutton5"));
        GtkSpinButton * spin6 = GTK_SPIN_BUTTON(gtk_builder_get_object(&amp;thisBuilder, "spinbutton6"));
        
        wl = sqrt(gtk_spin_button_get_value(spin4)*gtk_spin_button_get_value(spin4) + gtk_spin_button_get_value(spin5)*gtk_spin_button_get_value(spin5) + gtk_spin_button_get_value(spin6)*gtk_spin_button_get_value(spin6));
		if((wl &gt; 1 + l_eps) || (wl &lt; 1 - l_eps))
		{
			gtk_spin_button_set_value(spin4, gtk_spin_button_get_value(spin4) / wl);
			gtk_spin_button_set_value(spin5, gtk_spin_button_get_value(spin5) / wl);
			gtk_spin_button_set_value(spin6, gtk_spin_button_get_value(spin6) / wl);
		}
 	}
	
	void on_button</xsl:text><xsl:value-of select="($i*2)" /><xsl:text>_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_axis_xyz (GtkButton* button, gpointer userdata)
	{
 		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder &amp; thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1 = GTK_SPIN_BUTTON(gtk_builder_get_object(&amp;thisBuilder, "spinbuttonDown1"));
        GtkSpinButton * spin</xsl:text><xsl:value-of select="$i" /><xsl:text> = GTK_SPIN_BUTTON(gtk_builder_get_object(&amp;thisBuilder, "spinbutton</xsl:text><xsl:value-of select="$i" /><xsl:text>"));
        gtk_spin_button_set_value(spin</xsl:text><xsl:value-of select="$i" /><xsl:text>, gtk_spin_button_get_value(spin</xsl:text><xsl:value-of select="$i" /><xsl:text>) + gtk_spin_button_get_value(spinbuttonDown1));
        
        GtkSpinButton * spin4 = GTK_SPIN_BUTTON(gtk_builder_get_object(&amp;thisBuilder, "spinbutton4"));
        GtkSpinButton * spin5 = GTK_SPIN_BUTTON(gtk_builder_get_object(&amp;thisBuilder, "spinbutton5"));
        GtkSpinButton * spin6 = GTK_SPIN_BUTTON(gtk_builder_get_object(&amp;thisBuilder, "spinbutton6"));
        
        wl = sqrt(gtk_spin_button_get_value(spin4)*gtk_spin_button_get_value(spin4) + gtk_spin_button_get_value(spin5)*gtk_spin_button_get_value(spin5) + gtk_spin_button_get_value(spin6)*gtk_spin_button_get_value(spin6));
		if((wl &gt; 1 + l_eps) || (wl &lt; 1 - l_eps))
		{
			gtk_spin_button_set_value(spin4, gtk_spin_button_get_value(spin4) / wl);
			gtk_spin_button_set_value(spin5, gtk_spin_button_get_value(spin5) / wl);
			gtk_spin_button_set_value(spin6, gtk_spin_button_get_value(spin6) / wl);
		}
 	}   
</xsl:text>
 		</xsl:when>
 		<xsl:otherwise>
<xsl:text>
	void on_button</xsl:text><xsl:value-of select="($i*2)-1" /><xsl:text>_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_axis_xyz (GtkButton* button, gpointer userdata)
	{
 		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder &amp; thisBuilder = ((*ChoseEntry).getBuilder());
               
        GtkSpinButton * spin4 = GTK_SPIN_BUTTON(gtk_builder_get_object(&amp;thisBuilder, "spinbutton4"));
        GtkSpinButton * spin5 = GTK_SPIN_BUTTON(gtk_builder_get_object(&amp;thisBuilder, "spinbutton5"));
        GtkSpinButton * spin6 = GTK_SPIN_BUTTON(gtk_builder_get_object(&amp;thisBuilder, "spinbutton6"));
        
        wl = sqrt(gtk_spin_button_get_value(spin4)*gtk_spin_button_get_value(spin4) + gtk_spin_button_get_value(spin5)*gtk_spin_button_get_value(spin5) + gtk_spin_button_get_value(spin6)*gtk_spin_button_get_value(spin6));
		if((wl &gt; 1 + l_eps) || (wl &lt; 1 - l_eps))
		{
			gtk_spin_button_set_value(spin4, gtk_spin_button_get_value(spin4) / wl);
			gtk_spin_button_set_value(spin5, gtk_spin_button_get_value(spin5) / wl);
			gtk_spin_button_set_value(spin6, gtk_spin_button_get_value(spin6) / wl);
		}
 	}
	
	void on_button</xsl:text><xsl:value-of select="($i*2)" /><xsl:text>_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_axis_xyz (GtkButton* button, gpointer userdata)
	{
 		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder &amp; thisBuilder = ((*ChoseEntry).getBuilder());
        
        GtkSpinButton * spin4 = GTK_SPIN_BUTTON(gtk_builder_get_object(&amp;thisBuilder, "spinbutton4"));
        GtkSpinButton * spin5 = GTK_SPIN_BUTTON(gtk_builder_get_object(&amp;thisBuilder, "spinbutton5"));
        GtkSpinButton * spin6 = GTK_SPIN_BUTTON(gtk_builder_get_object(&amp;thisBuilder, "spinbutton6"));
        
        wl = sqrt(gtk_spin_button_get_value(spin4)*gtk_spin_button_get_value(spin4) + gtk_spin_button_get_value(spin5)*gtk_spin_button_get_value(spin5) + gtk_spin_button_get_value(spin6)*gtk_spin_button_get_value(spin6));
		if((wl &gt; 1 + l_eps) || (wl &lt; 1 - l_eps))
		{
			gtk_spin_button_set_value(spin4, gtk_spin_button_get_value(spin4) / wl);
			gtk_spin_button_set_value(spin5, gtk_spin_button_get_value(spin5) / wl);
			gtk_spin_button_set_value(spin6, gtk_spin_button_get_value(spin6) / wl);
		}
 	}   
</xsl:text>
 		</xsl:otherwise>		
 	</xsl:choose>
    </xsl:if>
	<!-- for loop --> 
       <xsl:if test="$i &lt;= $irp6EDPNumber">
          <xsl:call-template name="for.each.edp.irp6.axis.xyz.signals.cc">
              <xsl:with-param name="i">
                  <xsl:value-of select="$i + 1"/>
              </xsl:with-param>
              <xsl:with-param name="irp6EDPNumber">
                  <xsl:value-of select="$irp6EDPNumber"/>
              </xsl:with-param>
              <xsl:with-param name="fullName">
                  <xsl:value-of select="$fullName"/>
              </xsl:with-param>
          </xsl:call-template>
       </xsl:if>
</xsl:template>



<!-- signals handling file .h-->
<xsl:template name="irp6.axis.xyz.main.signals.h" match="*[substring(name(),1,4)='irp6']">
<xsl:variable name="name" select="name"/>
<xsl:document method="text" href="../signals/{$name}_axis_xyz_widget.h">

<xsl:text>
#ifndef __EDP_</xsl:text><xsl:value-of select="$name" /><xsl:text>_AXIS_XYZ
#define __EDP_</xsl:text><xsl:value-of select="$name" /><xsl:text>_AXIS_XYZ

#include &lt;iostream&gt;
#include &lt;vector&gt;

#include &lt;gtk/gtkbuilder.h&gt;
#include &lt;gtk/gtk.h&gt;

class edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>_axis_xyz
{
	public:

		edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>_axis_xyz(ui_widget_entry &amp;entry);
		~edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>_axis_xyz();


};

#endif /* __EDP_</xsl:text><xsl:value-of select="$name" /><xsl:text>_AXIS_XYZ */
</xsl:text>
</xsl:document>
</xsl:template>

</xsl:stylesheet>
