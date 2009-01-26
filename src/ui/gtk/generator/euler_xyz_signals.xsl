<?xml version="1.0" encoding="UTF-8"?>
<!--
MRROC++ GUI generator
Euler XYZ window callback signals
 -->
<xsl:stylesheet version="1.0" xmlns:xsl="http://www.w3.org/1999/XSL/Transform">
<xsl:output method="text"/>

<!-- signals handling file .cc-->
<xsl:template name="irp6.euler.xyz.main.signals.cc" match="*[substring(name(),1,4)='irp6']">
<xsl:variable name="name" select="name"/>
<xsl:variable name="fullName" select="fullName"/>
<xsl:variable name="irp6EDPNumber" select="euler_xyz"/>
<xsl:document method="text" href="../signals/{$name}_euler_xyz_widget.cc">


<xsl:text>
#include &lt;iostream&gt;
#include &lt;gtk/gtk.h&gt;
#include &lt;glib.h&gt;
#include "ui_model.h"
#include "</xsl:text><xsl:value-of select="$name" /><xsl:text>_euler_xyz_widget.h"

char buf[32];
gchar buffer[500];
double </xsl:text><xsl:value-of select="$name" /><xsl:text>_current_pos[</xsl:text><xsl:value-of select="$irp6EDPNumber" /><xsl:text>]; // pozycja biezaca
double </xsl:text><xsl:value-of select="$name" /><xsl:text>_desired_pos[</xsl:text><xsl:value-of select="$irp6EDPNumber" /><xsl:text>]; // pozycja zadana

edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>_euler_xyz::edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>_euler_xyz(ui_widget_entry &amp;entry) 
{
}

static edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>_euler_xyz *euler_xyz_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>;


extern "C"
{
	void on_arrow_button_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_euler_xyz (GtkButton* button, gpointer userdata)
	{
		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder &amp; thisBuilder = ((*ChoseEntry).getBuilder());
        
		</xsl:text><xsl:call-template name="irp6.euler.xyz.repeat.signals.cc.1">
    		<xsl:with-param name="euler_xyz" select="$irp6EDPNumber"/>
			<xsl:with-param name="i" select="1"/>
 		</xsl:call-template><xsl:text>
	}
	
	void on_read_button_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_euler_xyz (GtkButton* button, gpointer userdata)
		{
		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder &amp; thisBuilder = ((*ChoseEntry).getBuilder());
        
	</xsl:text><xsl:call-template name="irp6.euler.xyz.repeat.signals.cc.6">
    		<xsl:with-param name="irp6EDPNumber" select="$irp6EDPNumber"/>
			<xsl:with-param name="i" select="1"/>
 		</xsl:call-template><xsl:text>
 		
		if (robot</xsl:text><xsl:choose><xsl:when test="$name = 'conveyor'"></xsl:when><xsl:otherwise><xsl:text>->ecp</xsl:text></xsl:otherwise></xsl:choose><xsl:text>->get_EDP_pid()!=-1)
		{
			if (state.is_synchronised) // Czy robot jest zsynchronizowany?
			{
				if (!( robot->read_xyz_euler_zyz(</xsl:text><xsl:value-of select="$name" /><xsl:text>_current_pos))) // Odczyt polozenia walow silnikow
					printf("Blad w read motors\n");
					
</xsl:text><xsl:call-template name="irp6.euler.xyz.repeat.signals.cc.7">
    				<xsl:with-param name="irp6EDPNumber" select="$irp6EDPNumber"/>
    				<xsl:with-param name="name" select="$name"/>
					<xsl:with-param name="i" select="1"/>
 				</xsl:call-template><xsl:text>		
 				
				for (int i = 0; i &lt; </xsl:text><xsl:value-of select="$irp6EDPNumber" /><xsl:text>; i++)
				</xsl:text><xsl:value-of select="$name" /><xsl:text>_desired_pos[i] = </xsl:text><xsl:value-of select="$name" /><xsl:text>_current_pos[i];		
			}
			else
			{
				// Wygaszanie elementow przy niezsynchronizowanym robocie
				std::cout &lt;&lt; "nie jestem zsynchronizowany" &lt;&lt; std::endl;
			}
		}
	
	}
	
	void on_execute_button_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_euler_xyz (GtkButton* button, gpointer userdata)
	{
		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder &amp; thisBuilder = ((*ChoseEntry).getBuilder());
        
	</xsl:text><xsl:call-template name="irp6.euler.xyz.repeat.signals.cc.3">
    		<xsl:with-param name="irp6EDPNumber" select="$irp6EDPNumber"/>
			<xsl:with-param name="i" select="1"/>
 		</xsl:call-template><xsl:text>    

		if (robot</xsl:text><xsl:choose><xsl:when test="$name = 'conveyor'"></xsl:when><xsl:otherwise><xsl:text>->ecp</xsl:text></xsl:otherwise></xsl:choose><xsl:text>->get_EDP_pid()!=-1)
		{
			if (state.is_synchronised) {
	</xsl:text><xsl:call-template name="irp6.euler.xyz.repeat.signals.cc.8">
    		<xsl:with-param name="irp6EDPNumber" select="$irp6EDPNumber"/>
    		<xsl:with-param name="name" select="$name"/>
			<xsl:with-param name="i" select="1"/>
 		</xsl:call-template><xsl:text>    
			
			robot->move_xyz_euler_zyz(</xsl:text><xsl:value-of select="$name" /><xsl:text>_desired_pos);
			}
			 if (state.is_synchronised) {
	</xsl:text><xsl:call-template name="irp6.euler.xyz.repeat.signals.cc.9">
    		<xsl:with-param name="irp6EDPNumber" select="$irp6EDPNumber"/>
    		<xsl:with-param name="name" select="$name"/>
			<xsl:with-param name="i" select="1"/>
 		</xsl:call-template><xsl:text>  
			 }
		}
		on_read_button_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_euler_xyz (button, userdata);

	}
	
	void on_export_button_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_euler_xyz (GtkButton* button, gpointer userdata)
	{
		GtkEntry * entryConsole =  GTK_ENTRY(ui_model::instance().getUiGObject("entryConsole"));
     
 		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder &amp; thisBuilder = ((*ChoseEntry).getBuilder());
        
	</xsl:text><xsl:call-template name="irp6.euler.xyz.repeat.signals.cc.3">
    	<xsl:with-param name="irp6EDPNumber" select="$irp6EDPNumber"/>
		<xsl:with-param name="i" select="1"/>
 	</xsl:call-template><xsl:text>
 		sprintf(buffer, "edp_</xsl:text><xsl:value-of select="$name" /><xsl:text> XYZ EULER ZYZ position </xsl:text><xsl:call-template name="irp6.euler.xyz.repeat.signals.cc.4">
    		<xsl:with-param name="irp6EDPNumber" select="$irp6EDPNumber"/>
			<xsl:with-param name="i" select="1"/>
 		</xsl:call-template><xsl:text>" 
 		</xsl:text><xsl:call-template name="irp6.euler.xyz.repeat.signals.cc.5">
    		<xsl:with-param name="irp6EDPNumber" select="$irp6EDPNumber"/>
			<xsl:with-param name="i" select="1"/>
 		</xsl:call-template><xsl:text>);
 		  
 		gtk_entry_set_text (entryConsole, buffer);  
	}
	
	void on_import_button_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_euler_xyz (GtkButton* button, gpointer userdata)
	{
		GtkEntry * entryConsole =  GTK_ENTRY(ui_model::instance().getUiGObject("entryConsole"));
        
 		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder &amp; thisBuilder = ((*ChoseEntry).getBuilder());
		
		</xsl:text><xsl:call-template name="irp6.euler.xyz.repeat.signals.cc.2">
    		<xsl:with-param name="irp6EDPNumber" select="$irp6EDPNumber"/>
			<xsl:with-param name="i" select="1"/>
 		</xsl:call-template><xsl:text>  
 	}
	
	
	void ui_widget_init(ui_widget_entry &amp;entry) 
	{
		euler_xyz_</xsl:text><xsl:value-of select="$fullName" /><xsl:text> = new edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>_euler_xyz(entry);
		fprintf(stderr, "widget %s loaded\n", __FILE__);
		
		GtkButton * anyButton;
		gpointer userdata = &amp; entry;
		on_read_button_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_euler_xyz (anyButton, userdata);
	}

	void ui_widget_unload(void) 
	{
		if (euler_xyz_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>) 
		{
			delete euler_xyz_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>;
		}
		fprintf(stderr, "widget %s unloaded\n", __FILE__);
	}
	
</xsl:text>
		<xsl:call-template name="for.each.edp.irp6.euler.xyz.signals.cc">
    			<xsl:with-param name="irp6EDPNumber" select="$irp6EDPNumber"/>
    			<xsl:with-param name="fullName" select="$fullName"/>
			<xsl:with-param name="i" select="1"/>
 		</xsl:call-template>
<xsl:text>
}
</xsl:text>
</xsl:document>
<xsl:call-template name="irp6.euler.xyz.main.signals.h"/>
</xsl:template>

<!-- irp6 axis xyz handling signals .cc repeatable part -->
<xsl:template name="irp6.euler.xyz.repeat.signals.cc.1">
<xsl:param name="euler_xyz"/>
<xsl:param name="i"/>
	<xsl:if test="$i &lt;= $euler_xyz">
	<xsl:text>
        GtkEntry * entry</xsl:text><xsl:value-of select="$i" /><xsl:text> = GTK_ENTRY(gtk_builder_get_object(&amp;thisBuilder, "entry</xsl:text><xsl:value-of select="$i" /><xsl:text>"));
        GtkSpinButton * spin</xsl:text><xsl:value-of select="$i" /><xsl:text> = GTK_SPIN_BUTTON(gtk_builder_get_object(&amp;thisBuilder, "spinbutton</xsl:text><xsl:value-of select="$i" /><xsl:text>"));
        gtk_spin_button_set_value(spin</xsl:text><xsl:value-of select="$i" /><xsl:text>, atof(gtk_entry_get_text(entry</xsl:text><xsl:value-of select="$i" /><xsl:text>)));
	</xsl:text>
       </xsl:if>
	<!-- for loop --> 
       <xsl:if test="$i &lt;= $euler_xyz">
          <xsl:call-template name="irp6.euler.xyz.repeat.signals.cc.1">
              <xsl:with-param name="i">
                  <xsl:value-of select="$i + 1"/>
              </xsl:with-param>
              <xsl:with-param name="euler_xyz">
                  <xsl:value-of select="$euler_xyz"/>
              </xsl:with-param>
          </xsl:call-template>
       </xsl:if>
</xsl:template>

<!-- irp6 servo algorithm repeatable part -->
<xsl:template name="irp6.euler.xyz.repeat.signals.cc.2">
<xsl:param name="irp6EDPNumber"/>
<xsl:param name="i"/>
	<xsl:if test="$i &lt;= $irp6EDPNumber">
	<xsl:text>
 	    GtkSpinButton * spin</xsl:text><xsl:value-of select="$i" /><xsl:text> = GTK_SPIN_BUTTON(gtk_builder_get_object(&amp;thisBuilder, "spinbutton</xsl:text><xsl:value-of select="$i" /><xsl:text>"));
        gtk_spin_button_set_value(spin</xsl:text><xsl:value-of select="$i" /><xsl:text>, atof(gtk_entry_get_text(entryConsole)));
	</xsl:text>
       </xsl:if>
	<!-- for loop --> 
       <xsl:if test="$i &lt;= $irp6EDPNumber">
          <xsl:call-template name="irp6.euler.xyz.repeat.signals.cc.2">
              <xsl:with-param name="i">
                  <xsl:value-of select="$i + 1"/>
              </xsl:with-param>
              <xsl:with-param name="irp6EDPNumber">
                  <xsl:value-of select="$irp6EDPNumber"/>
              </xsl:with-param>
          </xsl:call-template>
       </xsl:if>
</xsl:template>

<!-- irp6 servo algorithm repeatable part -->
<xsl:template name="irp6.euler.xyz.repeat.signals.cc.3">
<xsl:param name="irp6EDPNumber"/>
<xsl:param name="i"/>
	<xsl:if test="$i &lt;= $irp6EDPNumber">
	<xsl:text>	GtkSpinButton * spin</xsl:text><xsl:value-of select="$i" /><xsl:text> = GTK_SPIN_BUTTON(gtk_builder_get_object(&amp;thisBuilder, "spinbutton</xsl:text><xsl:value-of select="$i" /><xsl:text>"));
 	</xsl:text>
       </xsl:if>
	<!-- for loop --> 
       <xsl:if test="$i &lt;= $irp6EDPNumber">
          <xsl:call-template name="irp6.euler.xyz.repeat.signals.cc.3">
              <xsl:with-param name="i">
                  <xsl:value-of select="$i + 1"/>
              </xsl:with-param>
              <xsl:with-param name="irp6EDPNumber">
                  <xsl:value-of select="$irp6EDPNumber"/>
              </xsl:with-param>
          </xsl:call-template>
       </xsl:if>
</xsl:template>

<!-- irp6 servo algorithm repeatable part -->
<xsl:template name="irp6.euler.xyz.repeat.signals.cc.4">
<xsl:param name="irp6EDPNumber"/>
<xsl:param name="i"/>
	<xsl:if test="$i &lt;= $irp6EDPNumber">
	<xsl:text> %f</xsl:text>
       </xsl:if>
	<!-- for loop --> 
       <xsl:if test="$i &lt;= $irp6EDPNumber">
          <xsl:call-template name="irp6.euler.xyz.repeat.signals.cc.4">
              <xsl:with-param name="i">
                  <xsl:value-of select="$i + 1"/>
              </xsl:with-param>
              <xsl:with-param name="irp6EDPNumber">
                  <xsl:value-of select="$irp6EDPNumber"/>
              </xsl:with-param>
          </xsl:call-template>
       </xsl:if>
</xsl:template>

<!-- irp6 servo algorithm repeatable part -->
<xsl:template name="irp6.euler.xyz.repeat.signals.cc.5">
<xsl:param name="irp6EDPNumber"/>
<xsl:param name="i"/>
	<xsl:if test="$i &lt;= $irp6EDPNumber">
	<xsl:text>, gtk_spin_button_get_value(spin</xsl:text><xsl:value-of select="$i" /><xsl:text>)</xsl:text>
       </xsl:if>
	<!-- for loop --> 
       <xsl:if test="$i &lt;= $irp6EDPNumber">
          <xsl:call-template name="irp6.euler.xyz.repeat.signals.cc.5">
              <xsl:with-param name="i">
                  <xsl:value-of select="$i + 1"/>
              </xsl:with-param>
              <xsl:with-param name="irp6EDPNumber">
                  <xsl:value-of select="$irp6EDPNumber"/>
              </xsl:with-param>
          </xsl:call-template>
       </xsl:if>
</xsl:template>

<!-- irp6 servo algorithm repeatable part -->
<xsl:template name="irp6.euler.xyz.repeat.signals.cc.6">
<xsl:param name="irp6EDPNumber"/>
<xsl:param name="i"/>
	<xsl:if test="$i &lt;= $irp6EDPNumber">
	<xsl:text>	GtkEntry * entry</xsl:text><xsl:value-of select="$i" /><xsl:text> = GTK_ENTRY(gtk_builder_get_object(&amp;thisBuilder, "entry</xsl:text><xsl:value-of select="$i" /><xsl:text>"));
	</xsl:text>
       </xsl:if>
	<!-- for loop --> 
       <xsl:if test="$i &lt;= $irp6EDPNumber">
          <xsl:call-template name="irp6.euler.xyz.repeat.signals.cc.6">
              <xsl:with-param name="i">
                  <xsl:value-of select="$i + 1"/>
              </xsl:with-param>
              <xsl:with-param name="irp6EDPNumber">
                  <xsl:value-of select="$irp6EDPNumber"/>
              </xsl:with-param>
          </xsl:call-template>
       </xsl:if>
</xsl:template>

<!-- irp6 servo algorithm repeatable part -->
<xsl:template name="irp6.euler.xyz.repeat.signals.cc.7">
<xsl:param name="irp6EDPNumber"/>
<xsl:param name="name"/>
<xsl:param name="i"/>
	<xsl:if test="$i &lt;= $irp6EDPNumber">
	<xsl:text>					snprintf (buf, sizeof(buf), "%.3f", </xsl:text><xsl:value-of select="$name" /><xsl:text>_current_pos[</xsl:text><xsl:value-of select="($i - 1)" /><xsl:text>]);
					gtk_entry_set_text(entry</xsl:text><xsl:value-of select="$i" /><xsl:text>, buf);
					</xsl:text><xsl:value-of select="$name" /><xsl:text>_desired_pos[</xsl:text><xsl:value-of select="($i - 1)" /><xsl:text>] = </xsl:text><xsl:value-of select="$name" /><xsl:text>_current_pos[</xsl:text><xsl:value-of select="($i - 1)" /><xsl:text>];				
</xsl:text>
       </xsl:if>
	<!-- for loop --> 
       <xsl:if test="$i &lt;= $irp6EDPNumber">
          <xsl:call-template name="irp6.euler.xyz.repeat.signals.cc.7">
              <xsl:with-param name="i">
                  <xsl:value-of select="$i + 1"/>
              </xsl:with-param>
              <xsl:with-param name="irp6EDPNumber">
                  <xsl:value-of select="$irp6EDPNumber"/>
              </xsl:with-param>
              <xsl:with-param name="name">
                  <xsl:value-of select="$name"/>
              </xsl:with-param>
          </xsl:call-template>
       </xsl:if>
</xsl:template>

<!-- irp6 servo algorithm repeatable part -->
<xsl:template name="irp6.euler.xyz.repeat.signals.cc.8">
<xsl:param name="irp6EDPNumber"/>
<xsl:param name="name"/>
<xsl:param name="i"/>
	<xsl:if test="$i &lt;= $irp6EDPNumber">
	<xsl:text>			</xsl:text><xsl:value-of select="$name" /><xsl:text>_desired_pos[</xsl:text><xsl:value-of select="($i - 1)" /><xsl:text>] = gtk_spin_button_get_value(spin</xsl:text><xsl:value-of select="$i" /><xsl:text>);
	</xsl:text>
       </xsl:if>
	<!-- for loop --> 
       <xsl:if test="$i &lt;= $irp6EDPNumber">
          <xsl:call-template name="irp6.euler.xyz.repeat.signals.cc.8">
              <xsl:with-param name="i">
                  <xsl:value-of select="$i + 1"/>
              </xsl:with-param>
              <xsl:with-param name="irp6EDPNumber">
                  <xsl:value-of select="$irp6EDPNumber"/>
              </xsl:with-param>
              <xsl:with-param name="name">
                  <xsl:value-of select="$name"/>
              </xsl:with-param>
          </xsl:call-template>
       </xsl:if>
</xsl:template>

<!-- irp6 servo algorithm repeatable part -->
<xsl:template name="irp6.euler.xyz.repeat.signals.cc.9">
<xsl:param name="irp6EDPNumber"/>
<xsl:param name="name"/>
<xsl:param name="i"/>
	<xsl:if test="$i &lt;= $irp6EDPNumber">
	<xsl:text>			gtk_spin_button_set_value(spin</xsl:text><xsl:value-of select="$i" /><xsl:text>, </xsl:text><xsl:value-of select="$name" /><xsl:text>_desired_pos[</xsl:text><xsl:value-of select="($i - 1)" /><xsl:text>]);
	</xsl:text>
       </xsl:if>
	<!-- for loop --> 
       <xsl:if test="$i &lt;= $irp6EDPNumber">
          <xsl:call-template name="irp6.euler.xyz.repeat.signals.cc.9">
              <xsl:with-param name="i">
                  <xsl:value-of select="$i + 1"/>
              </xsl:with-param>
              <xsl:with-param name="irp6EDPNumber">
                  <xsl:value-of select="$irp6EDPNumber"/>
              </xsl:with-param>
              <xsl:with-param name="name">
                  <xsl:value-of select="$name"/>
              </xsl:with-param>
          </xsl:call-template>
       </xsl:if>
</xsl:template>

<!-- handling signals .cc repeatable part -->
<xsl:template name="for.each.edp.irp6.euler.xyz.signals.cc">
<xsl:param name="irp6EDPNumber"/>
<xsl:param name="fullName"/>
<xsl:param name="i"/>
	<xsl:if test="$i &lt;= $irp6EDPNumber">
<xsl:text>
	void on_button</xsl:text><xsl:value-of select="($i*2)-1" /><xsl:text>_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_euler_xyz (GtkButton* button, gpointer userdata)
	{
 		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder &amp; thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1 = GTK_SPIN_BUTTON(gtk_builder_get_object(&amp;thisBuilder, "spinbuttonDown1"));
        GtkSpinButton * spin</xsl:text><xsl:value-of select="$i" /><xsl:text> = GTK_SPIN_BUTTON(gtk_builder_get_object(&amp;thisBuilder, "spinbutton</xsl:text><xsl:value-of select="$i" /><xsl:text>"));
        gtk_spin_button_set_value(spin</xsl:text><xsl:value-of select="$i" /><xsl:text>, gtk_spin_button_get_value(spin</xsl:text><xsl:value-of select="$i" /><xsl:text>) - gtk_spin_button_get_value(spinbuttonDown1));
 		
 		on_execute_button_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_euler_xyz (button, userdata);
 	} 
	
	void on_button</xsl:text><xsl:value-of select="($i*2)" /><xsl:text>_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_euler_xyz (GtkButton* button, gpointer userdata)
	{
 		ui_widget_entry * ChoseEntry = (ui_widget_entry *) userdata;
        GtkBuilder &amp; thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1 = GTK_SPIN_BUTTON(gtk_builder_get_object(&amp;thisBuilder, "spinbuttonDown1"));
        GtkSpinButton * spin</xsl:text><xsl:value-of select="$i" /><xsl:text> = GTK_SPIN_BUTTON(gtk_builder_get_object(&amp;thisBuilder, "spinbutton</xsl:text><xsl:value-of select="$i" /><xsl:text>"));
        gtk_spin_button_set_value(spin</xsl:text><xsl:value-of select="$i" /><xsl:text>, gtk_spin_button_get_value(spin</xsl:text><xsl:value-of select="$i" /><xsl:text>) + gtk_spin_button_get_value(spinbuttonDown1));
 		
 		on_execute_button_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_euler_xyz (button, userdata);
 	}   
</xsl:text>
    </xsl:if>
	<!-- for loop --> 
       <xsl:if test="$i &lt;= $irp6EDPNumber">
          <xsl:call-template name="for.each.edp.irp6.euler.xyz.signals.cc">
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
<xsl:template name="irp6.euler.xyz.main.signals.h" match="*[substring(name(),1,4)='irp6']">
<xsl:variable name="name" select="name"/>
<xsl:document method="text" href="../signals/{$name}_euler_xyz_widget.h">

<xsl:text>
#ifndef __EDP_</xsl:text><xsl:value-of select="$name" /><xsl:text>_EULER_XYZ
#define __EDP_</xsl:text><xsl:value-of select="$name" /><xsl:text>_EULER_XYZ

#include &lt;iostream&gt;
#include &lt;vector&gt;

#include &lt;gtk/gtkbuilder.h&gt;
#include &lt;gtk/gtk.h&gt;
#include "edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>_uimodule.h"

class edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>_euler_xyz
{
	public:

		edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>_euler_xyz(ui_widget_entry &amp;entry);
		~edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>_euler_xyz();


};

#endif /* __EDP_</xsl:text><xsl:value-of select="$name" /><xsl:text>_EULER_XYZ */
</xsl:text>
</xsl:document>
</xsl:template>

</xsl:stylesheet>
