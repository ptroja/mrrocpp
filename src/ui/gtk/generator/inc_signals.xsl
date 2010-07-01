<?xml version="1.0" encoding="UTF-8"?>
<!--
MRROC++ GUI generator
Inc window callback signals
 -->
<xsl:stylesheet version="1.0" xmlns:xsl="http://www.w3.org/1999/XSL/Transform">
<xsl:output method="text"/>

<!-- signals handling file .cc-->
<xsl:template name="irp6.inc.main.signals.cc" match="*[substring(name(),1,4)='irp6']">
<xsl:variable name="name" select="name"/>
<xsl:variable name="fullName" select="fullName"/>
<xsl:variable name="motorsNo" select="motorsNo"/>


extern "C"
{
	void on_arrow_button_clicked_<xsl:value-of select="$fullName" />_inc (GtkButton* button, gpointer userdata)
	{
		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder &amp; thisBuilder = ((*ChoseEntry).getBuilder());
        
		<xsl:call-template name="irp6.inc.repeat.signals.cc.arrow">
    		<xsl:with-param name="motorsNo" select="$motorsNo"/>
			<xsl:with-param name="i" select="1"/>
			<xsl:with-param name="name" select="$name"/>
 		</xsl:call-template>
	}
	
	void on_read_button_clicked_<xsl:value-of select="$fullName" />_inc (GtkButton* button, gpointer userdata)
	{
		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder &amp; thisBuilder = ((*ChoseEntry).getBuilder());
        
	<xsl:call-template name="irp6.inc.repeat.signals.cc.read.1">
    		<xsl:with-param name="motorsNo" select="$motorsNo"/>
			<xsl:with-param name="i" select="1"/>
			<xsl:with-param name="name" select="$name"/>
 		</xsl:call-template>
 		
		if (robot_<xsl:value-of select="$fullName" />->ecp->get_EDP_pid()!=-1)
		{
			if (state_<xsl:value-of select="$fullName" />.is_synchronised) // Czy robot jest zsynchronizowany?
			{
			    try {
					robot_<xsl:value-of select="$fullName" />->read_motors(<xsl:value-of select="$name" />_current_pos); // Odczyt polozenia walow silnikow
					
<xsl:call-template name="irp6.inc.repeat.signals.cc.read.2">
    				<xsl:with-param name="motorsNo" select="$motorsNo"/>
					<xsl:with-param name="name" select="$name"/>
					<xsl:with-param name="i" select="1"/>
 				</xsl:call-template>	
 				
	 				for (int i = 0; i &lt; <xsl:value-of select="$motorsNo" />; i++)
	 				<xsl:value-of select="$name" />_desired_pos[i] = <xsl:value-of select="$name" />_current_pos[i];
 			    }
 			    
 				<xsl:call-template name="catch" />
			}
			else
			{
				// Wygaszanie elementow przy niezsynchronizowanym robocie
				std::cout &lt;&lt; "Robot is not synchronized" &lt;&lt; std::endl;
			}
		}
	
	}
	
	void on_execute_button_clicked_<xsl:value-of select="$fullName" />_inc (GtkButton* button, gpointer userdata)
	{
		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder &amp; thisBuilder = ((*ChoseEntry).getBuilder());
        
	<xsl:call-template name="irp6.inc.repeat.signals.cc.execute.1">
    		<xsl:with-param name="motorsNo" select="$motorsNo"/>
			<xsl:with-param name="i" select="1"/>
			<xsl:with-param name="name" select="$name"/>
 		</xsl:call-template>    

		if (robot_<xsl:value-of select="$fullName" />->ecp->get_EDP_pid()!=-1)
		{
			if (state_<xsl:value-of select="$fullName" />.is_synchronised) {
	<xsl:call-template name="irp6.inc.repeat.signals.cc.execute.2">
    		<xsl:with-param name="motorsNo" select="$motorsNo"/>
			<xsl:with-param name="name" select="$name"/>
			<xsl:with-param name="i" select="1"/>
 		</xsl:call-template>    
			} else {
				for (int i = 0; i &lt; <xsl:value-of select="$motorsNo" />; i++)
				{
		        	<xsl:value-of select="$name" />_desired_pos[i] = 0.0;
	        	}
	   		}
			
			try {
				robot_<xsl:value-of select="$fullName" />->move_motors(<xsl:value-of select="$name" />_desired_pos);
				
				if (state_<xsl:value-of select="$fullName" />.is_synchronised) {
	<xsl:call-template name="irp6.inc.repeat.signals.cc.execute.3">
	    		<xsl:with-param name="motorsNo" select="$motorsNo"/>
				<xsl:with-param name="name" select="$name"/>
				<xsl:with-param name="i" select="1"/>
	 		</xsl:call-template>  
				}
			}
			<xsl:call-template name="catch" />
    
		}
		on_read_button_clicked_<xsl:value-of select="$fullName" />_inc (button, userdata);

	}
	
	void on_export_button_clicked_<xsl:value-of select="$fullName" />_inc (GtkButton* button, gpointer userdata)
	{
		GtkEntry * entryConsole =  GTK_ENTRY(ui_model::instance().getUiGObject("entryConsole"));
     
 		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder &amp; thisBuilder = ((*ChoseEntry).getBuilder());
        
	<xsl:call-template name="irp6.inc.repeat.signals.cc.execute.1">
    	<xsl:with-param name="motorsNo" select="$motorsNo"/>
		<xsl:with-param name="i" select="1"/>
		<xsl:with-param name="name" select="$name"/>
 	</xsl:call-template>
 		sprintf(buffer, "edp_<xsl:value-of select="$name" /> INCREMENT position <xsl:call-template name="irp6.inc.repeat.signals.cc.export.1">
    		<xsl:with-param name="motorsNo" select="$motorsNo"/>
			<xsl:with-param name="i" select="1"/>
			<xsl:with-param name="name" select="$name"/>
 		</xsl:call-template>" 
 		<xsl:call-template name="irp6.inc.repeat.signals.cc.export.2">
    		<xsl:with-param name="motorsNo" select="$motorsNo"/>
			<xsl:with-param name="i" select="1"/>
			<xsl:with-param name="name" select="$name"/>
 		</xsl:call-template>);
 		  
 		gtk_entry_set_text (entryConsole, buffer);  
	}
	
	void on_import_button_clicked_<xsl:value-of select="$fullName" />_inc (GtkButton* button, gpointer userdata)
	{
		GtkEntry * entryConsole =  GTK_ENTRY(ui_model::instance().getUiGObject("entryConsole"));
        
 		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder &amp; thisBuilder = ((*ChoseEntry).getBuilder());
		
        guint16 line_length = gtk_entry_get_text_length (entryConsole); 
        char line[line_length+1];
        strcpy(line, gtk_entry_get_text(entryConsole));
        
        char *ptr = line;
		<xsl:call-template name="irp6.inc.repeat.signals.cc.import.1">
    		<xsl:with-param name="motorsNo" select="$motorsNo"/>
			<xsl:with-param name="i" select="1"/>
			<xsl:with-param name="name" select="$name"/>
 		</xsl:call-template>  
 	}
	
	
	

		<xsl:call-template name="for.each.edp.irp6.inc.signals.cc">
    		<xsl:with-param name="motorsNo" select="$motorsNo"/>
    		<xsl:with-param name="fullName" select="$fullName"/>
			<xsl:with-param name="i" select="1"/>
			<xsl:with-param name="name" select="$name"/>
 		</xsl:call-template>

}

</xsl:template>

<!-- irp6 servo algorithm repeatable part -->
<xsl:template name="irp6.inc.repeat.signals.cc.arrow">
<xsl:param name="motorsNo"/>
<xsl:param name="i"/>
<xsl:param name="name"/>
	<xsl:if test="$i &lt;= $motorsNo">
	
        GtkEntry * entry<xsl:value-of select="$i" />_inc_<xsl:value-of select="$name" /> = GTK_ENTRY(gtk_builder_get_object(&amp;thisBuilder, "entry<xsl:value-of select="$i" />_inc_<xsl:value-of select="$name" />"));
        GtkSpinButton * spin<xsl:value-of select="$i" />_inc_<xsl:value-of select="$name" /> = GTK_SPIN_BUTTON(gtk_builder_get_object(&amp;thisBuilder, "spinbutton<xsl:value-of select="$i" />_inc_<xsl:value-of select="$name" />"));
        gtk_spin_button_set_value(spin<xsl:value-of select="$i" />_inc_<xsl:value-of select="$name" />, atof(gtk_entry_get_text(entry<xsl:value-of select="$i" />_inc_<xsl:value-of select="$name" />)));
	
       </xsl:if>
	<!-- for loop --> 
       <xsl:if test="$i &lt;= $motorsNo">
          <xsl:call-template name="irp6.inc.repeat.signals.cc.arrow">
              <xsl:with-param name="i">
                  <xsl:value-of select="$i + 1"/>
              </xsl:with-param>
              <xsl:with-param name="motorsNo">
                  <xsl:value-of select="$motorsNo"/>
              </xsl:with-param>
              <xsl:with-param name="name">
                  <xsl:value-of select="$name"/>
              </xsl:with-param>
          </xsl:call-template>
       </xsl:if>
</xsl:template>

<!-- irp6 servo algorithm repeatable part -->
<xsl:template name="irp6.inc.repeat.signals.cc.import.1">
<xsl:param name="motorsNo"/>
<xsl:param name="i"/>
<xsl:param name="name"/>
	<xsl:if test="$i &lt;= $motorsNo">
	
 	    GtkSpinButton * spin<xsl:value-of select="$i" />_inc_<xsl:value-of select="$name" /> = GTK_SPIN_BUTTON(gtk_builder_get_object(&amp;thisBuilder, "spinbutton<xsl:value-of select="$i" />_inc_<xsl:value-of select="$name" />"));
        gtk_spin_button_set_value(spin<xsl:value-of select="$i" />_inc_<xsl:value-of select="$name" />, strtod(ptr, &amp;ptr));
	
       </xsl:if>
	<!-- for loop --> 
       <xsl:if test="$i &lt;= $motorsNo">
          <xsl:call-template name="irp6.inc.repeat.signals.cc.import.1">
              <xsl:with-param name="i">
                  <xsl:value-of select="$i + 1"/>
              </xsl:with-param>
              <xsl:with-param name="motorsNo">
                  <xsl:value-of select="$motorsNo"/>
              </xsl:with-param>
              <xsl:with-param name="name">
                  <xsl:value-of select="$name"/>
              </xsl:with-param>
          </xsl:call-template>
       </xsl:if>
</xsl:template>

<!-- irp6 servo algorithm repeatable part -->
<xsl:template name="irp6.inc.repeat.signals.cc.execute.1">
<xsl:param name="motorsNo"/>
<xsl:param name="i"/>
<xsl:param name="name"/>
	<xsl:if test="$i &lt;= $motorsNo">
		GtkSpinButton * spin<xsl:value-of select="$i" />_inc_<xsl:value-of select="$name" /> = GTK_SPIN_BUTTON(gtk_builder_get_object(&amp;thisBuilder, "spinbutton<xsl:value-of select="$i" />_inc_<xsl:value-of select="$name" />"));
	</xsl:if>
	<!-- for loop --> 
       <xsl:if test="$i &lt;= $motorsNo">
          <xsl:call-template name="irp6.inc.repeat.signals.cc.execute.1">
              <xsl:with-param name="i">
                  <xsl:value-of select="$i + 1"/>
              </xsl:with-param>
              <xsl:with-param name="motorsNo">
                  <xsl:value-of select="$motorsNo"/>
              </xsl:with-param>
              <xsl:with-param name="name">
                  <xsl:value-of select="$name"/>
              </xsl:with-param>
          </xsl:call-template>
       </xsl:if>
</xsl:template>

<!-- irp6 servo algorithm repeatable part -->
<xsl:template name="irp6.inc.repeat.signals.cc.export.1">
<xsl:param name="motorsNo"/>
<xsl:param name="i"/>
<xsl:param name="name"/>
	<xsl:if test="$i &lt;= $motorsNo"> %.3f</xsl:if>
	<!-- for loop --> 
       <xsl:if test="$i &lt;= $motorsNo">
          <xsl:call-template name="irp6.inc.repeat.signals.cc.export.1">
              <xsl:with-param name="i">
                  <xsl:value-of select="$i + 1"/>
              </xsl:with-param>
              <xsl:with-param name="motorsNo">
                  <xsl:value-of select="$motorsNo"/>
              </xsl:with-param>
              <xsl:with-param name="name">
                  <xsl:value-of select="$name"/>
              </xsl:with-param>
          </xsl:call-template>
       </xsl:if>
</xsl:template>

<!-- irp6 servo algorithm repeatable part -->
<xsl:template name="irp6.inc.repeat.signals.cc.export.2">
<xsl:param name="motorsNo"/>
<xsl:param name="i"/>
<xsl:param name="name"/>
	<xsl:if test="$i &lt;= $motorsNo">
	, gtk_spin_button_get_value(spin<xsl:value-of select="$i" />_inc_<xsl:value-of select="$name" />)
       </xsl:if>
	<!-- for loop --> 
       <xsl:if test="$i &lt;= $motorsNo">
          <xsl:call-template name="irp6.inc.repeat.signals.cc.export.2">
              <xsl:with-param name="i">
                  <xsl:value-of select="$i + 1"/>
              </xsl:with-param>
              <xsl:with-param name="motorsNo">
                  <xsl:value-of select="$motorsNo"/>
              </xsl:with-param>
              <xsl:with-param name="name">
                  <xsl:value-of select="$name"/>
              </xsl:with-param>
          </xsl:call-template>
       </xsl:if>
</xsl:template>

<!-- irp6 servo algorithm repeatable part -->
<xsl:template name="irp6.inc.repeat.signals.cc.read.1">
<xsl:param name="motorsNo"/>
<xsl:param name="i"/>
<xsl:param name="name"/>
	<xsl:if test="$i &lt;= $motorsNo">
		GtkEntry * entry<xsl:value-of select="$i" />_inc_<xsl:value-of select="$name" /> = GTK_ENTRY(gtk_builder_get_object(&amp;thisBuilder, "entry<xsl:value-of select="$i" />_inc_<xsl:value-of select="$name" />"));
	
       </xsl:if>
	<!-- for loop --> 
       <xsl:if test="$i &lt;= $motorsNo">
          <xsl:call-template name="irp6.inc.repeat.signals.cc.read.1">
              <xsl:with-param name="i">
                  <xsl:value-of select="$i + 1"/>
              </xsl:with-param>
              <xsl:with-param name="motorsNo">
                  <xsl:value-of select="$motorsNo"/>
              </xsl:with-param>
              <xsl:with-param name="name">
                  <xsl:value-of select="$name"/>
              </xsl:with-param>
          </xsl:call-template>
       </xsl:if>
</xsl:template>

<!-- irp6 servo algorithm repeatable part -->
<xsl:template name="irp6.inc.repeat.signals.cc.read.2">
<xsl:param name="motorsNo"/>
<xsl:param name="name"/>
<xsl:param name="i"/>
	<xsl:if test="$i &lt;= $motorsNo">
						snprintf (buf, sizeof(buf), "%.3f", <xsl:value-of select="$name" />_current_pos[<xsl:value-of select="($i - 1)" />]);
					gtk_entry_set_text(entry<xsl:value-of select="$i" />_inc_<xsl:value-of select="$name" />, buf);
					<xsl:value-of select="$name" />_desired_pos[<xsl:value-of select="($i - 1)" />] = <xsl:value-of select="$name" />_current_pos[<xsl:value-of select="($i - 1)" />];				

       </xsl:if>
	<!-- for loop --> 
       <xsl:if test="$i &lt;= $motorsNo">
          <xsl:call-template name="irp6.inc.repeat.signals.cc.read.2">
              <xsl:with-param name="i">
                  <xsl:value-of select="$i + 1"/>
              </xsl:with-param>
              <xsl:with-param name="motorsNo">
                  <xsl:value-of select="$motorsNo"/>
              </xsl:with-param>
              <xsl:with-param name="name">
                  <xsl:value-of select="$name"/>
              </xsl:with-param>
          </xsl:call-template>
       </xsl:if>
</xsl:template>

<!-- irp6 servo algorithm repeatable part -->
<xsl:template name="irp6.inc.repeat.signals.cc.execute.2">
<xsl:param name="motorsNo"/>
<xsl:param name="name"/>
<xsl:param name="i"/>
	<xsl:if test="$i &lt;= $motorsNo">
				<xsl:value-of select="$name" />_desired_pos[<xsl:value-of select="($i - 1)" />] = gtk_spin_button_get_value(spin<xsl:value-of select="$i" />_inc_<xsl:value-of select="$name" />);
	</xsl:if>
	<!-- for loop --> 
       <xsl:if test="$i &lt;= $motorsNo">
          <xsl:call-template name="irp6.inc.repeat.signals.cc.execute.2">
              <xsl:with-param name="i">
                  <xsl:value-of select="$i + 1"/>
              </xsl:with-param>
              <xsl:with-param name="motorsNo">
                  <xsl:value-of select="$motorsNo"/>
              </xsl:with-param>
              <xsl:with-param name="name">
                  <xsl:value-of select="$name"/>
              </xsl:with-param>
          </xsl:call-template>
       </xsl:if>
</xsl:template>

<!-- irp6 servo algorithm repeatable part -->
<xsl:template name="irp6.inc.repeat.signals.cc.execute.3">
<xsl:param name="motorsNo"/>
<xsl:param name="name"/>
<xsl:param name="i"/>
	<xsl:if test="$i &lt;= $motorsNo">
				gtk_spin_button_set_value(spin<xsl:value-of select="$i" />_inc_<xsl:value-of select="$name" />, <xsl:value-of select="$name" />_desired_pos[<xsl:value-of select="($i - 1)" />]);
	</xsl:if>
	<!-- for loop --> 
       <xsl:if test="$i &lt;= $motorsNo">
          <xsl:call-template name="irp6.inc.repeat.signals.cc.execute.3">
              <xsl:with-param name="i">
                  <xsl:value-of select="$i + 1"/>
              </xsl:with-param>
              <xsl:with-param name="motorsNo">
                  <xsl:value-of select="$motorsNo"/>
              </xsl:with-param>
              <xsl:with-param name="name">
                  <xsl:value-of select="$name"/>
              </xsl:with-param>
          </xsl:call-template>
       </xsl:if>
</xsl:template>

<!-- handling signals .cc repeatable part -->
<xsl:template name="for.each.edp.irp6.inc.signals.cc">
<xsl:param name="motorsNo"/>
<xsl:param name="fullName"/>
<xsl:param name="i"/>
<xsl:param name="name"/>
	<xsl:if test="$i &lt;= $motorsNo">

	void on_button<xsl:value-of select="($i*2)-1" />_clicked_<xsl:value-of select="$fullName" />_inc (GtkButton* button, gpointer userdata)
	{
 		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder &amp; thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1_inc_<xsl:value-of select="$name" /> = GTK_SPIN_BUTTON(gtk_builder_get_object(&amp;thisBuilder, "spinbuttonDown1_inc_<xsl:value-of select="$name" />"));
        GtkSpinButton * spin<xsl:value-of select="$i" />_inc_<xsl:value-of select="$name" /> = GTK_SPIN_BUTTON(gtk_builder_get_object(&amp;thisBuilder, "spinbutton<xsl:value-of select="$i" />_inc_<xsl:value-of select="$name" />"));
        gtk_spin_button_set_value(spin<xsl:value-of select="$i" />_inc_<xsl:value-of select="$name" />, gtk_spin_button_get_value(spin<xsl:value-of select="$i" />_inc_<xsl:value-of select="$name" />) - gtk_spin_button_get_value(spinbuttonDown1_inc_<xsl:value-of select="$name" />));
 	
		on_execute_button_clicked_<xsl:value-of select="$fullName" />_inc (button, userdata); 	
 	}
	
	void on_button<xsl:value-of select="($i*2)" />_clicked_<xsl:value-of select="$fullName" />_inc (GtkButton* button, gpointer userdata)
	{
 		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder &amp; thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1_inc_<xsl:value-of select="$name" /> = GTK_SPIN_BUTTON(gtk_builder_get_object(&amp;thisBuilder, "spinbuttonDown1_inc_<xsl:value-of select="$name" />"));
        GtkSpinButton * spin<xsl:value-of select="$i" />_inc_<xsl:value-of select="$name" /> = GTK_SPIN_BUTTON(gtk_builder_get_object(&amp;thisBuilder, "spinbutton<xsl:value-of select="$i" />_inc_<xsl:value-of select="$name" />"));
        gtk_spin_button_set_value(spin<xsl:value-of select="$i" />_inc_<xsl:value-of select="$name" />, gtk_spin_button_get_value(spin<xsl:value-of select="$i" />_inc_<xsl:value-of select="$name" />) + gtk_spin_button_get_value(spinbuttonDown1_inc_<xsl:value-of select="$name" />));
 	
 		on_execute_button_clicked_<xsl:value-of select="$fullName" />_inc (button, userdata);
 	}    

    </xsl:if>
	<!-- for loop --> 
       <xsl:if test="$i &lt;= $motorsNo">
          <xsl:call-template name="for.each.edp.irp6.inc.signals.cc">
              <xsl:with-param name="i">
                  <xsl:value-of select="$i + 1"/>
              </xsl:with-param>
              <xsl:with-param name="motorsNo">
                  <xsl:value-of select="$motorsNo"/>
              </xsl:with-param>
              <xsl:with-param name="fullName">
                  <xsl:value-of select="$fullName"/>
              </xsl:with-param>
              <xsl:with-param name="name">
                  <xsl:value-of select="$name"/>
              </xsl:with-param>
          </xsl:call-template>
       </xsl:if>
</xsl:template>

</xsl:stylesheet>
