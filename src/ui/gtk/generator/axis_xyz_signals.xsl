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
<xsl:variable name="axis_xyz" select="axis_xyz"/>

<xsl:text>

extern "C"
{
	void on_arrow_button_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_axis_xyz (GtkButton* button, gpointer userdata)
	{
		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder &amp; thisBuilder = ((*ChoseEntry).getBuilder());
        
		</xsl:text><xsl:call-template name="irp6.axis.xyz.repeat.signals.cc.arrow">
    		<xsl:with-param name="axis_xyz" select="$axis_xyz"/>
			<xsl:with-param name="i" select="1"/>
			<xsl:with-param name="name" select="$name"/>
 		</xsl:call-template><xsl:text>
	}
	
	void on_read_button_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_axis_xyz (GtkButton* button, gpointer userdata)
		{
		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder &amp; thisBuilder = ((*ChoseEntry).getBuilder());
        
	</xsl:text><xsl:call-template name="irp6.axis.xyz.repeat.signals.read.1">
    		<xsl:with-param name="axis_xyz" select="$axis_xyz"/>
			<xsl:with-param name="i" select="1"/>
			<xsl:with-param name="name" select="$name"/>
 		</xsl:call-template><xsl:text>
 		
		if (robot_</xsl:text><xsl:value-of select="$fullName" /><xsl:text></xsl:text><xsl:choose><xsl:when test="$name = 'conveyor'"></xsl:when><xsl:otherwise><xsl:text>->ecp</xsl:text></xsl:otherwise></xsl:choose><xsl:text>->get_EDP_pid()!=-1)
		{
			if (state_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>.is_synchronised) // Czy robot jest zsynchronizowany?
			{
				if (!( robot_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>->read_xyz_angle_axis(</xsl:text><xsl:value-of select="$name" /><xsl:text>_current_pos_a))) // Odczyt polozenia walow silnikow
					printf("Blad w read motors\n");
					
				alfa = sqrt(</xsl:text><xsl:value-of select="$name" /><xsl:text>_current_pos_a[3]*</xsl:text><xsl:value-of select="$name" /><xsl:text>_current_pos_a[3]
				+</xsl:text><xsl:value-of select="$name" /><xsl:text>_current_pos_a[4]*</xsl:text><xsl:value-of select="$name" /><xsl:text>_current_pos_a[4]
				+</xsl:text><xsl:value-of select="$name" /><xsl:text>_current_pos_a[5]*</xsl:text><xsl:value-of select="$name" /><xsl:text>_current_pos_a[5]);
					
</xsl:text><xsl:call-template name="irp6.axis.xyz.repeat.signals.read.2">
    				<xsl:with-param name="axis_xyz" select="$axis_xyz"/>
					<xsl:with-param name="name" select="$name"/>
					<xsl:with-param name="i" select="1"/>
 				</xsl:call-template><xsl:text>				
			}
			else
			{
				// Wygaszanie elementow przy niezsynchronizowanym robocie
				std::cout &lt;&lt; "Robot is not synchronized" &lt;&lt; std::endl;
			}
		}
	
	}
	
	void on_execute_button_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_axis_xyz (GtkButton* button, gpointer userdata)
		{
		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder &amp; thisBuilder = ((*ChoseEntry).getBuilder());
        
	</xsl:text><xsl:call-template name="irp6.axis.xyz.repeat.signals.execute.1">
    		<xsl:with-param name="axis_xyz" select="$axis_xyz"/>
			<xsl:with-param name="i" select="1"/>
			<xsl:with-param name="name" select="$name"/>
 		</xsl:call-template><xsl:text>    

		if (robot_</xsl:text><xsl:value-of select="$fullName" /><xsl:text></xsl:text><xsl:choose><xsl:when test="$name = 'conveyor'"></xsl:when><xsl:otherwise><xsl:text>->ecp</xsl:text></xsl:otherwise></xsl:choose><xsl:text>->get_EDP_pid()!=-1)
		{
		
			wl = sqrt(gtk_spin_button_get_value(spin4_axis_xyz_</xsl:text><xsl:value-of select="$name" /><xsl:text>)*gtk_spin_button_get_value(spin4_axis_xyz_</xsl:text><xsl:value-of select="$name" /><xsl:text>) + gtk_spin_button_get_value(spin5_axis_xyz_</xsl:text><xsl:value-of select="$name" /><xsl:text>)*gtk_spin_button_get_value(spin5_axis_xyz_</xsl:text><xsl:value-of select="$name" /><xsl:text>) + gtk_spin_button_get_value(spin6_axis_xyz_</xsl:text><xsl:value-of select="$name" /><xsl:text>)*gtk_spin_button_get_value(spin6_axis_xyz_</xsl:text><xsl:value-of select="$name" /><xsl:text>));
			if((wl &gt; 1 + l_eps) || (wl &lt; 1 - l_eps))
			{
				gtk_spin_button_set_value(spin4_axis_xyz_</xsl:text><xsl:value-of select="$name" /><xsl:text>, gtk_spin_button_get_value(spin4_axis_xyz_</xsl:text><xsl:value-of select="$name" /><xsl:text>) / wl);
				gtk_spin_button_set_value(spin5_axis_xyz_</xsl:text><xsl:value-of select="$name" /><xsl:text>, gtk_spin_button_get_value(spin5_axis_xyz_</xsl:text><xsl:value-of select="$name" /><xsl:text>) / wl);
				gtk_spin_button_set_value(spin6_axis_xyz_</xsl:text><xsl:value-of select="$name" /><xsl:text>, gtk_spin_button_get_value(spin6_axis_xyz_</xsl:text><xsl:value-of select="$name" /><xsl:text>) / wl);
			}		
		
	</xsl:text><xsl:call-template name="irp6.axis.ts.repeat.signals.cc.execute.2">
    		<xsl:with-param name="axis_xyz" select="$axis_xyz"/>
			<xsl:with-param name="name" select="$name"/>
			<xsl:with-param name="i" select="1"/>
 		</xsl:call-template><xsl:text>    
 		
 			// przepisanie parametrow ruchu do postaci rozkazu w formie XYZ_ANGLE_AXIS
			for(int i=3; i&lt;</xsl:text><xsl:value-of select="$axis_xyz" /><xsl:text>; i++)
			{
					</xsl:text><xsl:value-of select="$name" /><xsl:text>_desired_pos_a[i] *= </xsl:text><xsl:value-of select="$name" /><xsl:text>_desired_pos_a[6];
			}
			
			robot_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>->move_xyz_angle_axis(</xsl:text><xsl:value-of select="$name" /><xsl:text>_desired_pos_a);
			
			 if (state_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>.is_synchronised) {
	</xsl:text><xsl:call-template name="irp6.axis.ts.repeat.signals.cc.execute.3">
    		<xsl:with-param name="axis_xyz" select="$axis_xyz"/>
			<xsl:with-param name="name" select="$name"/>
			<xsl:with-param name="i" select="1"/>
 		</xsl:call-template><xsl:text>  
			 }
		}
		on_read_button_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_axis_xyz (button, userdata);

	}
	
</xsl:text>
		<xsl:call-template name="for.each.edp.irp6.axis.xyz.signals.cc">
    		<xsl:with-param name="axis_xyz" select="$axis_xyz"/>
    		<xsl:with-param name="fullName" select="$fullName"/>
			<xsl:with-param name="name" select="$name"/>
			<xsl:with-param name="i" select="1"/>
 		</xsl:call-template>
<xsl:text>
}
</xsl:text>
</xsl:template>

<!-- irp6 axis xyz handling signals .cc repeatable part -->
<xsl:template name="irp6.axis.xyz.repeat.signals.cc.arrow">
<xsl:param name="axis_xyz"/>
<xsl:param name="i"/>
<xsl:param name="name"/>
	<xsl:if test="$i &lt;= $axis_xyz">
	<xsl:text>
        GtkEntry * entry</xsl:text><xsl:value-of select="$i" /><xsl:text>_axis_xyz_</xsl:text><xsl:value-of select="$name" /><xsl:text> = GTK_ENTRY(gtk_builder_get_object(&amp;thisBuilder, "entry</xsl:text><xsl:value-of select="$i" /><xsl:text>_axis_xyz_</xsl:text><xsl:value-of select="$name" /><xsl:text>"));
        GtkSpinButton * spin</xsl:text><xsl:value-of select="$i" /><xsl:text>_axis_xyz_</xsl:text><xsl:value-of select="$name" /><xsl:text> = GTK_SPIN_BUTTON(gtk_builder_get_object(&amp;thisBuilder, "spinbutton</xsl:text><xsl:value-of select="$i" /><xsl:text>_axis_xyz_</xsl:text><xsl:value-of select="$name" /><xsl:text>"));
        gtk_spin_button_set_value(spin</xsl:text><xsl:value-of select="$i" /><xsl:text>_axis_xyz_</xsl:text><xsl:value-of select="$name" /><xsl:text>, atof(gtk_entry_get_text(entry</xsl:text><xsl:value-of select="$i" /><xsl:text>_axis_xyz_</xsl:text><xsl:value-of select="$name" /><xsl:text>)));
	</xsl:text>
       </xsl:if>
	<!-- for loop --> 
       <xsl:if test="$i &lt;= $axis_xyz">
          <xsl:call-template name="irp6.axis.xyz.repeat.signals.cc.arrow">
              <xsl:with-param name="i">
                  <xsl:value-of select="$i + 1"/>
              </xsl:with-param>
              <xsl:with-param name="axis_xyz">
                  <xsl:value-of select="$axis_xyz"/>
              </xsl:with-param>
              <xsl:with-param name="name">
                  <xsl:value-of select="$name"/>
              </xsl:with-param>
          </xsl:call-template>
       </xsl:if>
</xsl:template>

<!-- irp6 servo algorithm repeatable part -->
<xsl:template name="irp6.axis.xyz.repeat.signals.execute.1">
<xsl:param name="axis_xyz"/>
<xsl:param name="i"/>
<xsl:param name="name"/>
	<xsl:if test="$i &lt;= $axis_xyz">
	<xsl:text>	GtkSpinButton * spin</xsl:text><xsl:value-of select="$i" /><xsl:text>_axis_xyz_</xsl:text><xsl:value-of select="$name" /><xsl:text> = GTK_SPIN_BUTTON(gtk_builder_get_object(&amp;thisBuilder, "spinbutton</xsl:text><xsl:value-of select="$i" /><xsl:text>_axis_xyz_</xsl:text><xsl:value-of select="$name" /><xsl:text>"));
 	</xsl:text>
       </xsl:if>
	<!-- for loop --> 
       <xsl:if test="$i &lt;= $axis_xyz">
          <xsl:call-template name="irp6.axis.xyz.repeat.signals.execute.1">
              <xsl:with-param name="i">
                  <xsl:value-of select="$i + 1"/>
              </xsl:with-param>
              <xsl:with-param name="axis_xyz">
                  <xsl:value-of select="$axis_xyz"/>
              </xsl:with-param>
              <xsl:with-param name="name">
                  <xsl:value-of select="$name"/>
              </xsl:with-param>
          </xsl:call-template>
       </xsl:if>
</xsl:template>

<!-- irp6 servo algorithm repeatable part -->
<xsl:template name="irp6.axis.xyz.repeat.signals.read.1">
<xsl:param name="axis_xyz"/>
<xsl:param name="i"/>
<xsl:param name="name"/>
	<xsl:if test="$i &lt;= $axis_xyz">
	<xsl:text>	GtkEntry * entry</xsl:text><xsl:value-of select="$i" /><xsl:text>_axis_xyz_</xsl:text><xsl:value-of select="$name" /><xsl:text> = GTK_ENTRY(gtk_builder_get_object(&amp;thisBuilder, "entry</xsl:text><xsl:value-of select="$i" /><xsl:text>_axis_xyz_</xsl:text><xsl:value-of select="$name" /><xsl:text>"));
	</xsl:text>
       </xsl:if>
	<!-- for loop --> 
       <xsl:if test="$i &lt;= $axis_xyz">
          <xsl:call-template name="irp6.axis.xyz.repeat.signals.read.1">
              <xsl:with-param name="i">
                  <xsl:value-of select="$i + 1"/>
              </xsl:with-param>
              <xsl:with-param name="axis_xyz">
                  <xsl:value-of select="$axis_xyz"/>
              </xsl:with-param>
              <xsl:with-param name="name">
                  <xsl:value-of select="$name"/>
              </xsl:with-param>
          </xsl:call-template>
       </xsl:if>
</xsl:template>

<!-- irp6 servo algorithm repeatable part -->
<xsl:template name="irp6.axis.xyz.repeat.signals.read.2">
<xsl:param name="axis_xyz"/>
<xsl:param name="name"/>
<xsl:param name="i"/>
	<xsl:if test="$i &lt;= $axis_xyz">
	<xsl:choose>
		<xsl:when test="$i &lt;= 3">
	<xsl:text>					snprintf (buf, sizeof(buf), "%.3f", </xsl:text><xsl:value-of select="$name" /><xsl:text>_current_pos_a[</xsl:text><xsl:value-of select="($i - 1)" /><xsl:text>]);
					gtk_entry_set_text(entry</xsl:text><xsl:value-of select="$i" /><xsl:text>_axis_xyz_</xsl:text><xsl:value-of select="$name" /><xsl:text>, buf);
					</xsl:text><xsl:value-of select="$name" /><xsl:text>_desired_pos_a[</xsl:text><xsl:value-of select="($i - 1)" /><xsl:text>] = </xsl:text><xsl:value-of select="$name" /><xsl:text>_current_pos_a[</xsl:text><xsl:value-of select="($i - 1)" /><xsl:text>];				
</xsl:text>
 		</xsl:when>
 		<xsl:when test="$i &gt;= 8">
	<xsl:text>					snprintf (buf, sizeof(buf), "%.3f", </xsl:text><xsl:value-of select="$name" /><xsl:text>_current_pos_a[</xsl:text><xsl:value-of select="($i - 1)" /><xsl:text>]);
					gtk_entry_set_text(entry</xsl:text><xsl:value-of select="$i" /><xsl:text>_axis_xyz_</xsl:text><xsl:value-of select="$name" /><xsl:text>, buf);
					</xsl:text><xsl:value-of select="$name" /><xsl:text>_desired_pos_a[</xsl:text><xsl:value-of select="($i - 1)" /><xsl:text>] = </xsl:text><xsl:value-of select="$name" /><xsl:text>_current_pos_a[</xsl:text><xsl:value-of select="($i - 1)" /><xsl:text>];				
</xsl:text>
 		</xsl:when>
<xsl:when test="$i = 7">
<xsl:text>					snprintf (buf, sizeof(buf), "%.3f", alfa);
					gtk_entry_set_text(entry</xsl:text><xsl:value-of select="$i" /><xsl:text>_axis_xyz_</xsl:text><xsl:value-of select="$name" /><xsl:text>, buf);
					</xsl:text><xsl:value-of select="$name" /><xsl:text>_desired_pos_a[</xsl:text><xsl:value-of select="($i - 1)" /><xsl:text>] = alfa;							
</xsl:text>
 		</xsl:when>
 		<xsl:otherwise>
<xsl:text>					snprintf (buf, sizeof(buf), "%.3f", </xsl:text><xsl:value-of select="$name" /><xsl:text>_current_pos_a[</xsl:text><xsl:value-of select="($i - 1)" /><xsl:text>]/alfa);
					gtk_entry_set_text(entry</xsl:text><xsl:value-of select="$i" /><xsl:text>_axis_xyz_</xsl:text><xsl:value-of select="$name" /><xsl:text>, buf);
					</xsl:text><xsl:value-of select="$name" /><xsl:text>_desired_pos_a[</xsl:text><xsl:value-of select="($i - 1)" /><xsl:text>] = </xsl:text><xsl:value-of select="$name" /><xsl:text>_current_pos_a[</xsl:text><xsl:value-of select="($i - 1)" /><xsl:text>]/alfa;
</xsl:text>
 		</xsl:otherwise>		
 	</xsl:choose>
	
       </xsl:if>
	<!-- for loop --> 
       <xsl:if test="$i &lt;= $axis_xyz">
          <xsl:call-template name="irp6.axis.xyz.repeat.signals.read.2">
              <xsl:with-param name="i">
                  <xsl:value-of select="$i + 1"/>
              </xsl:with-param>
              <xsl:with-param name="axis_xyz">
                  <xsl:value-of select="$axis_xyz"/>
              </xsl:with-param>
              <xsl:with-param name="name">
                  <xsl:value-of select="$name"/>
              </xsl:with-param>
          </xsl:call-template>
       </xsl:if>
</xsl:template>

<!-- irp6 servo algorithm repeatable part -->
<xsl:template name="irp6.axis.ts.repeat.signals.cc.execute.2">
<xsl:param name="axis_xyz"/>
<xsl:param name="name"/>
<xsl:param name="i"/>
	<xsl:if test="$i &lt;= $axis_xyz">
	<xsl:text>			</xsl:text><xsl:value-of select="$name" /><xsl:text>_desired_pos_a[</xsl:text><xsl:value-of select="($i - 1)" /><xsl:text>] = gtk_spin_button_get_value(spin</xsl:text><xsl:value-of select="$i" /><xsl:text>_axis_xyz_</xsl:text><xsl:value-of select="$name" /><xsl:text>);
	</xsl:text>
       </xsl:if>
	<!-- for loop --> 
       <xsl:if test="$i &lt;= $axis_xyz">
          <xsl:call-template name="irp6.axis.ts.repeat.signals.cc.execute.2">
              <xsl:with-param name="i">
                  <xsl:value-of select="$i + 1"/>
              </xsl:with-param>
              <xsl:with-param name="axis_xyz">
                  <xsl:value-of select="$axis_xyz"/>
              </xsl:with-param>
              <xsl:with-param name="name">
                  <xsl:value-of select="$name"/>
              </xsl:with-param>
          </xsl:call-template>
       </xsl:if>
</xsl:template>

<!-- irp6 servo algorithm repeatable part -->
<xsl:template name="irp6.axis.ts.repeat.signals.cc.execute.3">
<xsl:param name="axis_xyz"/>
<xsl:param name="name"/>
<xsl:param name="i"/>
	<xsl:if test="$i &lt;= $axis_xyz">
	<xsl:text>			gtk_spin_button_set_value(spin</xsl:text><xsl:value-of select="$i" /><xsl:text>_axis_xyz_</xsl:text><xsl:value-of select="$name" /><xsl:text>, </xsl:text><xsl:value-of select="$name" /><xsl:text>_desired_pos_a[</xsl:text><xsl:value-of select="($i - 1)" /><xsl:text>]);
	</xsl:text>
       </xsl:if>
	<!-- for loop --> 
       <xsl:if test="$i &lt;= $axis_xyz">
          <xsl:call-template name="irp6.axis.ts.repeat.signals.cc.execute.3">
              <xsl:with-param name="i">
                  <xsl:value-of select="$i + 1"/>
              </xsl:with-param>
              <xsl:with-param name="axis_xyz">
                  <xsl:value-of select="$axis_xyz"/>
              </xsl:with-param>
              <xsl:with-param name="name">
                  <xsl:value-of select="$name"/>
              </xsl:with-param>
          </xsl:call-template>
       </xsl:if>
</xsl:template>

<!-- handling signals .cc repeatable part -->
<xsl:template name="for.each.edp.irp6.axis.xyz.signals.cc">
<xsl:param name="axis_xyz"/>
<xsl:param name="fullName"/>
<xsl:param name="i"/>
<xsl:param name="name"/>
	<xsl:if test="$i &lt;= $axis_xyz">
	<xsl:choose>
		<xsl:when test="$i &lt;= 3">
<xsl:text>
	void on_button</xsl:text><xsl:value-of select="($i*2)-1" /><xsl:text>_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_axis_xyz (GtkButton* button, gpointer userdata)
	{
 		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder &amp; thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1_axis_xyz_</xsl:text><xsl:value-of select="$name" /><xsl:text> = GTK_SPIN_BUTTON(gtk_builder_get_object(&amp;thisBuilder, "spinbuttonDown1_axis_xyz_</xsl:text><xsl:value-of select="$name" /><xsl:text>"));
        GtkSpinButton * spin</xsl:text><xsl:value-of select="$i" /><xsl:text>_axis_xyz_</xsl:text><xsl:value-of select="$name" /><xsl:text> = GTK_SPIN_BUTTON(gtk_builder_get_object(&amp;thisBuilder, "spinbutton</xsl:text><xsl:value-of select="$i" /><xsl:text>_axis_xyz_</xsl:text><xsl:value-of select="$name" /><xsl:text>"));
        gtk_spin_button_set_value(spin</xsl:text><xsl:value-of select="$i" /><xsl:text>_axis_xyz_</xsl:text><xsl:value-of select="$name" /><xsl:text>, gtk_spin_button_get_value(spin</xsl:text><xsl:value-of select="$i" /><xsl:text>_axis_xyz_</xsl:text><xsl:value-of select="$name" /><xsl:text>) - gtk_spin_button_get_value(spinbuttonDown1_axis_xyz_</xsl:text><xsl:value-of select="$name" /><xsl:text>));
		
		on_execute_button_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_axis_xyz (button, userdata);
 	}
	
	void on_button</xsl:text><xsl:value-of select="($i*2)" /><xsl:text>_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_axis_xyz (GtkButton* button, gpointer userdata)
	{
 		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder &amp; thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1_axis_xyz_</xsl:text><xsl:value-of select="$name" /><xsl:text> = GTK_SPIN_BUTTON(gtk_builder_get_object(&amp;thisBuilder, "spinbuttonDown1_axis_xyz_</xsl:text><xsl:value-of select="$name" /><xsl:text>"));
        GtkSpinButton * spin</xsl:text><xsl:value-of select="$i" /><xsl:text>_axis_xyz_</xsl:text><xsl:value-of select="$name" /><xsl:text> = GTK_SPIN_BUTTON(gtk_builder_get_object(&amp;thisBuilder, "spinbutton</xsl:text><xsl:value-of select="$i" /><xsl:text>_axis_xyz_</xsl:text><xsl:value-of select="$name" /><xsl:text>"));
        gtk_spin_button_set_value(spin</xsl:text><xsl:value-of select="$i" /><xsl:text>_axis_xyz_</xsl:text><xsl:value-of select="$name" /><xsl:text>, gtk_spin_button_get_value(spin</xsl:text><xsl:value-of select="$i" /><xsl:text>_axis_xyz_</xsl:text><xsl:value-of select="$name" /><xsl:text>) + gtk_spin_button_get_value(spinbuttonDown1_axis_xyz_</xsl:text><xsl:value-of select="$name" /><xsl:text>));
		
		on_execute_button_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_axis_xyz (button, userdata);
}   
</xsl:text>
 		</xsl:when>
 		<xsl:when test="$i &gt;= 7">
<xsl:text>
	void on_button</xsl:text><xsl:value-of select="($i*2)-1" /><xsl:text>_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_axis_xyz (GtkButton* button, gpointer userdata)
	{
 		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder &amp; thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1_axis_xyz_</xsl:text><xsl:value-of select="$name" /><xsl:text> = GTK_SPIN_BUTTON(gtk_builder_get_object(&amp;thisBuilder, "spinbuttonDown1_axis_xyz_</xsl:text><xsl:value-of select="$name" /><xsl:text>"));
        GtkSpinButton * spin</xsl:text><xsl:value-of select="$i" /><xsl:text>_axis_xyz_</xsl:text><xsl:value-of select="$name" /><xsl:text> = GTK_SPIN_BUTTON(gtk_builder_get_object(&amp;thisBuilder, "spinbutton</xsl:text><xsl:value-of select="$i" /><xsl:text>_axis_xyz_</xsl:text><xsl:value-of select="$name" /><xsl:text>"));
        gtk_spin_button_set_value(spin</xsl:text><xsl:value-of select="$i" /><xsl:text>_axis_xyz_</xsl:text><xsl:value-of select="$name" /><xsl:text>, gtk_spin_button_get_value(spin</xsl:text><xsl:value-of select="$i" /><xsl:text>_axis_xyz_</xsl:text><xsl:value-of select="$name" /><xsl:text>) - gtk_spin_button_get_value(spinbuttonDown1_axis_xyz_</xsl:text><xsl:value-of select="$name" /><xsl:text>));
		
		on_execute_button_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_axis_xyz (button, userdata);
	}
	
	void on_button</xsl:text><xsl:value-of select="($i*2)" /><xsl:text>_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_axis_xyz (GtkButton* button, gpointer userdata)
	{
 		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder &amp; thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1_axis_xyz_</xsl:text><xsl:value-of select="$name" /><xsl:text> = GTK_SPIN_BUTTON(gtk_builder_get_object(&amp;thisBuilder, "spinbuttonDown1_axis_xyz_</xsl:text><xsl:value-of select="$name" /><xsl:text>"));
        GtkSpinButton * spin</xsl:text><xsl:value-of select="$i" /><xsl:text>_axis_xyz_</xsl:text><xsl:value-of select="$name" /><xsl:text> = GTK_SPIN_BUTTON(gtk_builder_get_object(&amp;thisBuilder, "spinbutton</xsl:text><xsl:value-of select="$i" /><xsl:text>_axis_xyz_</xsl:text><xsl:value-of select="$name" /><xsl:text>"));
        gtk_spin_button_set_value(spin</xsl:text><xsl:value-of select="$i" /><xsl:text>_axis_xyz_</xsl:text><xsl:value-of select="$name" /><xsl:text>, gtk_spin_button_get_value(spin</xsl:text><xsl:value-of select="$i" /><xsl:text>_axis_xyz_</xsl:text><xsl:value-of select="$name" /><xsl:text>) + gtk_spin_button_get_value(spinbuttonDown1_axis_xyz_</xsl:text><xsl:value-of select="$name" /><xsl:text>));
        
		on_execute_button_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_axis_xyz (button, userdata);
	}   
</xsl:text>
 		</xsl:when>
 		<xsl:otherwise>
<xsl:text>
	void on_button</xsl:text><xsl:value-of select="($i*2)-1" /><xsl:text>_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_axis_xyz (GtkButton* button, gpointer userdata)
	{
		on_execute_button_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_axis_xyz (button, userdata);
	}
	
	void on_button</xsl:text><xsl:value-of select="($i*2)" /><xsl:text>_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_axis_xyz (GtkButton* button, gpointer userdata)
	{
		on_execute_button_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_axis_xyz (button, userdata);
	}  
</xsl:text>
 		</xsl:otherwise>		
 	</xsl:choose>
    </xsl:if>
	<!-- for loop --> 
       <xsl:if test="$i &lt;= $axis_xyz">
          <xsl:call-template name="for.each.edp.irp6.axis.xyz.signals.cc">
              <xsl:with-param name="i">
                  <xsl:value-of select="$i + 1"/>
              </xsl:with-param>
              <xsl:with-param name="axis_xyz">
                  <xsl:value-of select="$axis_xyz"/>
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
