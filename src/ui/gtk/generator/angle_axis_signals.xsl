<?xml version="1.0" encoding="UTF-8"?>
<!--
MRROC++ GUI generator
Axis XYZ window callback signals
 -->
<xsl:stylesheet version="1.0" xmlns:xsl="http://www.w3.org/1999/XSL/Transform">
<xsl:output method="text"/>

<!-- signals handling file .cc-->
<xsl:template name="irp6.xyz_angle_axis.main.signals.cc" match="*[substring(name(),1,4)='irp6']">
<xsl:variable name="name" select="name"/>
<xsl:variable name="fullName" select="fullName"/>
<xsl:variable name="xyz_angle_axis" select="xyz_angle_axis"/>



extern "C"
{
	void on_arrow_button_clicked_<xsl:value-of select="$fullName" />_xyz_angle_axis (GtkButton* button, gpointer userdata)
	{
		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder &amp; thisBuilder = ((*ChoseEntry).getBuilder());
        
		<xsl:call-template name="irp6.xyz_angle_axis.repeat.signals.cc.arrow">
    		<xsl:with-param name="xyz_angle_axis" select="$xyz_angle_axis"/>
			<xsl:with-param name="i" select="1"/>
			<xsl:with-param name="name" select="$name"/>
 		</xsl:call-template>
	}
	
	void on_read_button_clicked_<xsl:value-of select="$fullName" />_xyz_angle_axis (GtkButton* button, gpointer userdata)
		{
		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder &amp; thisBuilder = ((*ChoseEntry).getBuilder());
        
	<xsl:call-template name="irp6.xyz_angle_axis.repeat.signals.read.1">
    		<xsl:with-param name="xyz_angle_axis" select="$xyz_angle_axis"/>
			<xsl:with-param name="i" select="1"/>
			<xsl:with-param name="name" select="$name"/>
 		</xsl:call-template>
 		
		if (robot_<xsl:value-of select="$fullName" />->ecp->get_EDP_pid()!=-1)
		{
			if (state_<xsl:value-of select="$fullName" />.is_synchronised) // Czy robot jest zsynchronizowany?
			{

                try {
				    robot_<xsl:value-of select="$fullName" />->read_xyz_angle_axis(<xsl:value-of select="$name" />_current_pos_a); // Odczyt polozenia walow silnikow
						
					alfa = sqrt(<xsl:value-of select="$name" />_current_pos_a[3]*<xsl:value-of select="$name" />_current_pos_a[3]
					+<xsl:value-of select="$name" />_current_pos_a[4]*<xsl:value-of select="$name" />_current_pos_a[4]
					+<xsl:value-of select="$name" />_current_pos_a[5]*<xsl:value-of select="$name" />_current_pos_a[5]);
					
<xsl:call-template name="irp6.xyz_angle_axis.repeat.signals.read.2">
    				<xsl:with-param name="xyz_angle_axis" select="$xyz_angle_axis"/>
					<xsl:with-param name="name" select="$name"/>
					<xsl:with-param name="i" select="1"/>
</xsl:call-template>
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
	
	void on_execute_button_clicked_<xsl:value-of select="$fullName" />_xyz_angle_axis (GtkButton* button, gpointer userdata)
		{
		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder &amp; thisBuilder = ((*ChoseEntry).getBuilder());
        
		<xsl:call-template name="irp6.xyz_angle_axis.repeat.signals.execute.1">
    		<xsl:with-param name="xyz_angle_axis" select="$xyz_angle_axis"/>
			<xsl:with-param name="i" select="1"/>
			<xsl:with-param name="name" select="$name"/>
 		</xsl:call-template>    

		if (robot_<xsl:value-of select="$fullName" />->ecp->get_EDP_pid()!=-1)
		{
			const double kx = gtk_spin_button_get_value(spin4_xyz_angle_axis_<xsl:value-of select="$name" />);
			const double ky = gtk_spin_button_get_value(spin5_xyz_angle_axis_<xsl:value-of select="$name" />);
			const double kz = gtk_spin_button_get_value(spin6_xyz_angle_axis_<xsl:value-of select="$name" />);
		
			const double wl = sqrt(kx*kx + ky*ky + kz*kz);
			
			if((wl &gt; 1 + l_eps) || (wl &lt; 1 - l_eps))
			{
				gtk_spin_button_set_value(spin4_xyz_angle_axis_<xsl:value-of select="$name" />, gtk_spin_button_get_value(spin4_xyz_angle_axis_<xsl:value-of select="$name" />) / wl);
				gtk_spin_button_set_value(spin5_xyz_angle_axis_<xsl:value-of select="$name" />, gtk_spin_button_get_value(spin5_xyz_angle_axis_<xsl:value-of select="$name" />) / wl);
				gtk_spin_button_set_value(spin6_xyz_angle_axis_<xsl:value-of select="$name" />, gtk_spin_button_get_value(spin6_xyz_angle_axis_<xsl:value-of select="$name" />) / wl);
			}		
		
			<xsl:call-template name="irp6.axis.ts.repeat.signals.cc.execute.2">
	    		<xsl:with-param name="xyz_angle_axis" select="$xyz_angle_axis"/>
				<xsl:with-param name="name" select="$name"/>
				<xsl:with-param name="i" select="1"/>
 			</xsl:call-template>    
 		
 			// przepisanie parametrow ruchu do postaci rozkazu w formie XYZ_ANGLE_AXIS
			for(int i=3; i&lt;6; i++)
			{
					<xsl:value-of select="$name" />_desired_pos_a[i] *= <xsl:value-of select="$name" />_desired_pos_a[6];
			}
			<xsl:value-of select="$name" />_desired_pos_a[6] = <xsl:value-of select="$name" />_desired_pos_a[7];
			
			try {
				robot_<xsl:value-of select="$fullName" />->move_xyz_angle_axis(<xsl:value-of select="$name" />_desired_pos_a);
				
				if (state_<xsl:value-of select="$fullName" />.is_synchronised) {
					on_read_button_clicked_<xsl:value-of select="$fullName" />_xyz_angle_axis(NULL, userdata);
				}
			}
		<xsl:call-template name="catch" />
		}
		on_read_button_clicked_<xsl:value-of select="$fullName" />_xyz_angle_axis (button, userdata);

	}
	

		<xsl:call-template name="for.each.edp.irp6.xyz_angle_axis.signals.cc">
    		<xsl:with-param name="xyz_angle_axis" select="$xyz_angle_axis"/>
    		<xsl:with-param name="fullName" select="$fullName"/>
			<xsl:with-param name="name" select="$name"/>
			<xsl:with-param name="i" select="1"/>
 		</xsl:call-template>

	}

</xsl:template>

<!-- irp6 axis xyz handling signals .cc repeatable part -->
<xsl:template name="irp6.xyz_angle_axis.repeat.signals.cc.arrow">
<xsl:param name="xyz_angle_axis"/>
<xsl:param name="i"/>
<xsl:param name="name"/>
	<xsl:if test="$i &lt;= $xyz_angle_axis">
	
        GtkEntry * entry<xsl:value-of select="$i" />_xyz_angle_axis_<xsl:value-of select="$name" /> = GTK_ENTRY(gtk_builder_get_object(&amp;thisBuilder, "entry<xsl:value-of select="$i" />_xyz_angle_axis_<xsl:value-of select="$name" />"));
        GtkSpinButton * spin<xsl:value-of select="$i" />_xyz_angle_axis_<xsl:value-of select="$name" /> = GTK_SPIN_BUTTON(gtk_builder_get_object(&amp;thisBuilder, "spinbutton<xsl:value-of select="$i" />_xyz_angle_axis_<xsl:value-of select="$name" />"));
        gtk_spin_button_set_value(spin<xsl:value-of select="$i" />_xyz_angle_axis_<xsl:value-of select="$name" />, atof(gtk_entry_get_text(entry<xsl:value-of select="$i" />_xyz_angle_axis_<xsl:value-of select="$name" />)));
	
       </xsl:if>
	<!-- for loop --> 
       <xsl:if test="$i &lt;= $xyz_angle_axis">
          <xsl:call-template name="irp6.xyz_angle_axis.repeat.signals.cc.arrow">
              <xsl:with-param name="i">
                  <xsl:value-of select="$i + 1"/>
              </xsl:with-param>
              <xsl:with-param name="xyz_angle_axis">
                  <xsl:value-of select="$xyz_angle_axis"/>
              </xsl:with-param>
              <xsl:with-param name="name">
                  <xsl:value-of select="$name"/>
              </xsl:with-param>
          </xsl:call-template>
       </xsl:if>
</xsl:template>

<!-- irp6 servo algorithm repeatable part -->
<xsl:template name="irp6.xyz_angle_axis.repeat.signals.execute.1">
<xsl:param name="xyz_angle_axis"/>
<xsl:param name="i"/>
<xsl:param name="name"/>
	<xsl:if test="$i &lt;= $xyz_angle_axis">
		GtkSpinButton * spin<xsl:value-of select="$i" />_xyz_angle_axis_<xsl:value-of select="$name" /> = GTK_SPIN_BUTTON(gtk_builder_get_object(&amp;thisBuilder, "spinbutton<xsl:value-of select="$i" />_xyz_angle_axis_<xsl:value-of select="$name" />"));
 	
       </xsl:if>
	<!-- for loop --> 
       <xsl:if test="$i &lt;= $xyz_angle_axis">
          <xsl:call-template name="irp6.xyz_angle_axis.repeat.signals.execute.1">
              <xsl:with-param name="i">
                  <xsl:value-of select="$i + 1"/>
              </xsl:with-param>
              <xsl:with-param name="xyz_angle_axis">
                  <xsl:value-of select="$xyz_angle_axis"/>
              </xsl:with-param>
              <xsl:with-param name="name">
                  <xsl:value-of select="$name"/>
              </xsl:with-param>
          </xsl:call-template>
       </xsl:if>
</xsl:template>

<!-- irp6 servo algorithm repeatable part -->
<xsl:template name="irp6.xyz_angle_axis.repeat.signals.read.1">
<xsl:param name="xyz_angle_axis"/>
<xsl:param name="i"/>
<xsl:param name="name"/>
	<xsl:if test="$i &lt;= $xyz_angle_axis">
		GtkEntry * entry<xsl:value-of select="$i" />_xyz_angle_axis_<xsl:value-of select="$name" /> = GTK_ENTRY(gtk_builder_get_object(&amp;thisBuilder, "entry<xsl:value-of select="$i" />_xyz_angle_axis_<xsl:value-of select="$name" />"));
	
       </xsl:if>
	<!-- for loop --> 
       <xsl:if test="$i &lt;= $xyz_angle_axis">
          <xsl:call-template name="irp6.xyz_angle_axis.repeat.signals.read.1">
              <xsl:with-param name="i">
                  <xsl:value-of select="$i + 1"/>
              </xsl:with-param>
              <xsl:with-param name="xyz_angle_axis">
                  <xsl:value-of select="$xyz_angle_axis"/>
              </xsl:with-param>
              <xsl:with-param name="name">
                  <xsl:value-of select="$name"/>
              </xsl:with-param>
          </xsl:call-template>
       </xsl:if>
</xsl:template>

<!-- irp6 servo algorithm repeatable part -->
<xsl:template name="irp6.xyz_angle_axis.repeat.signals.read.2">
<xsl:param name="xyz_angle_axis"/>
<xsl:param name="name"/>
<xsl:param name="i"/>
	<xsl:if test="$i &lt;= $xyz_angle_axis">
	<xsl:choose>
		<xsl:when test="$i &lt;= 3">
					snprintf (buf, sizeof(buf), "%.3f", <xsl:value-of select="$name" />_current_pos_a[<xsl:value-of select="($i - 1)" />]);
					gtk_entry_set_text(entry<xsl:value-of select="$i" />_xyz_angle_axis_<xsl:value-of select="$name" />, buf);
					<xsl:value-of select="$name" />_desired_pos_a[<xsl:value-of select="($i - 1)" />] = <xsl:value-of select="$name" />_current_pos_a[<xsl:value-of select="($i - 1)" />];				

 		</xsl:when>
 		<xsl:when test="$i &gt;= 8">
					snprintf (buf, sizeof(buf), "%.3f", <xsl:value-of select="$name" />_current_pos_a[<xsl:value-of select="($i - 2)" />]);
					gtk_entry_set_text(entry<xsl:value-of select="$i" />_xyz_angle_axis_<xsl:value-of select="$name" />, buf);
					<xsl:value-of select="$name" />_desired_pos_a[<xsl:value-of select="($i - 2)" />] = <xsl:value-of select="$name" />_current_pos_a[<xsl:value-of select="($i - 2)" />];				

 		</xsl:when>
		<xsl:when test="$i = 7">
					snprintf (buf, sizeof(buf), "%.3f", alfa);
					gtk_entry_set_text(entry<xsl:value-of select="$i" />_xyz_angle_axis_<xsl:value-of select="$name" />, buf);
					<xsl:value-of select="$name" />_desired_pos_a[<xsl:value-of select="($i - 1)" />] = alfa;							

 		</xsl:when>
 		<xsl:otherwise>
					snprintf (buf, sizeof(buf), "%.3f", <xsl:value-of select="$name" />_current_pos_a[<xsl:value-of select="($i - 1)" />]/alfa);
					gtk_entry_set_text(entry<xsl:value-of select="$i" />_xyz_angle_axis_<xsl:value-of select="$name" />, buf);
					<xsl:value-of select="$name" />_desired_pos_a[<xsl:value-of select="($i - 1)" />] = <xsl:value-of select="$name" />_current_pos_a[<xsl:value-of select="($i - 1)" />]/alfa;

 		</xsl:otherwise>		
 	</xsl:choose>
	</xsl:if>
	<!-- for loop --> 
       <xsl:if test="$i &lt;= $xyz_angle_axis">
          <xsl:call-template name="irp6.xyz_angle_axis.repeat.signals.read.2">
              <xsl:with-param name="i">
                  <xsl:value-of select="$i + 1"/>
              </xsl:with-param>
              <xsl:with-param name="xyz_angle_axis">
                  <xsl:value-of select="$xyz_angle_axis"/>
              </xsl:with-param>
              <xsl:with-param name="name">
                  <xsl:value-of select="$name"/>
              </xsl:with-param>
          </xsl:call-template>
       </xsl:if>
</xsl:template>

<!-- irp6 servo algorithm repeatable part -->
<xsl:template name="irp6.axis.ts.repeat.signals.cc.execute.2">
<xsl:param name="xyz_angle_axis"/>
<xsl:param name="name"/>
<xsl:param name="i"/>
	<xsl:if test="$i &lt;= $xyz_angle_axis">
		<xsl:value-of select="$name" />_desired_pos_a[<xsl:value-of select="($i - 1)" />] = gtk_spin_button_get_value(spin<xsl:value-of select="$i" />_xyz_angle_axis_<xsl:value-of select="$name" />);	
	</xsl:if>
	<!-- for loop --> 
       <xsl:if test="$i &lt;= $xyz_angle_axis">
          <xsl:call-template name="irp6.axis.ts.repeat.signals.cc.execute.2">
              <xsl:with-param name="i">
                  <xsl:value-of select="$i + 1"/>
              </xsl:with-param>
              <xsl:with-param name="xyz_angle_axis">
                  <xsl:value-of select="$xyz_angle_axis"/>
              </xsl:with-param>
              <xsl:with-param name="name">
                  <xsl:value-of select="$name"/>
              </xsl:with-param>
          </xsl:call-template>
       </xsl:if>
</xsl:template>

<!-- handling signals .cc repeatable part -->
<xsl:template name="for.each.edp.irp6.xyz_angle_axis.signals.cc">
<xsl:param name="xyz_angle_axis"/>
<xsl:param name="fullName"/>
<xsl:param name="i"/>
<xsl:param name="name"/>
	<xsl:if test="$i &lt;= $xyz_angle_axis">
	<xsl:choose>
		<xsl:when test="$i &lt;= 3">

	void on_button<xsl:value-of select="($i*2)-1" />_clicked_<xsl:value-of select="$fullName" />_xyz_angle_axis (GtkButton* button, gpointer userdata)
	{
 		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder &amp; thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1_xyz_angle_axis_<xsl:value-of select="$name" /> = GTK_SPIN_BUTTON(gtk_builder_get_object(&amp;thisBuilder, "spinbuttonDown1_xyz_angle_axis_<xsl:value-of select="$name" />"));
        GtkSpinButton * spin<xsl:value-of select="$i" />_xyz_angle_axis_<xsl:value-of select="$name" /> = GTK_SPIN_BUTTON(gtk_builder_get_object(&amp;thisBuilder, "spinbutton<xsl:value-of select="$i" />_xyz_angle_axis_<xsl:value-of select="$name" />"));
        gtk_spin_button_set_value(spin<xsl:value-of select="$i" />_xyz_angle_axis_<xsl:value-of select="$name" />, gtk_spin_button_get_value(spin<xsl:value-of select="$i" />_xyz_angle_axis_<xsl:value-of select="$name" />) - gtk_spin_button_get_value(spinbuttonDown1_xyz_angle_axis_<xsl:value-of select="$name" />));
		
		on_execute_button_clicked_<xsl:value-of select="$fullName" />_xyz_angle_axis (button, userdata);
 	}
	
	void on_button<xsl:value-of select="($i*2)" />_clicked_<xsl:value-of select="$fullName" />_xyz_angle_axis (GtkButton* button, gpointer userdata)
	{
 		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder &amp; thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1_xyz_angle_axis_<xsl:value-of select="$name" /> = GTK_SPIN_BUTTON(gtk_builder_get_object(&amp;thisBuilder, "spinbuttonDown1_xyz_angle_axis_<xsl:value-of select="$name" />"));
        GtkSpinButton * spin<xsl:value-of select="$i" />_xyz_angle_axis_<xsl:value-of select="$name" /> = GTK_SPIN_BUTTON(gtk_builder_get_object(&amp;thisBuilder, "spinbutton<xsl:value-of select="$i" />_xyz_angle_axis_<xsl:value-of select="$name" />"));
        gtk_spin_button_set_value(spin<xsl:value-of select="$i" />_xyz_angle_axis_<xsl:value-of select="$name" />, gtk_spin_button_get_value(spin<xsl:value-of select="$i" />_xyz_angle_axis_<xsl:value-of select="$name" />) + gtk_spin_button_get_value(spinbuttonDown1_xyz_angle_axis_<xsl:value-of select="$name" />));
		
		on_execute_button_clicked_<xsl:value-of select="$fullName" />_xyz_angle_axis (button, userdata);
	}   

 		</xsl:when>
 		<xsl:when test="$i &gt;= 7">

	void on_button<xsl:value-of select="($i*2)-1" />_clicked_<xsl:value-of select="$fullName" />_xyz_angle_axis (GtkButton* button, gpointer userdata)
	{
 		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder &amp; thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1_xyz_angle_axis_<xsl:value-of select="$name" /> = GTK_SPIN_BUTTON(gtk_builder_get_object(&amp;thisBuilder, "spinbuttonDown1_xyz_angle_axis_<xsl:value-of select="$name" />"));
        GtkSpinButton * spin<xsl:value-of select="$i" />_xyz_angle_axis_<xsl:value-of select="$name" /> = GTK_SPIN_BUTTON(gtk_builder_get_object(&amp;thisBuilder, "spinbutton<xsl:value-of select="$i" />_xyz_angle_axis_<xsl:value-of select="$name" />"));
        gtk_spin_button_set_value(spin<xsl:value-of select="$i" />_xyz_angle_axis_<xsl:value-of select="$name" />, gtk_spin_button_get_value(spin<xsl:value-of select="$i" />_xyz_angle_axis_<xsl:value-of select="$name" />) - gtk_spin_button_get_value(spinbuttonDown1_xyz_angle_axis_<xsl:value-of select="$name" />));
		
		on_execute_button_clicked_<xsl:value-of select="$fullName" />_xyz_angle_axis (button, userdata);
	}
	
	void on_button<xsl:value-of select="($i*2)" />_clicked_<xsl:value-of select="$fullName" />_xyz_angle_axis (GtkButton* button, gpointer userdata)
	{
 		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder &amp; thisBuilder = ((*ChoseEntry).getBuilder());
       
        GtkSpinButton * spinbuttonDown1_xyz_angle_axis_<xsl:value-of select="$name" /> = GTK_SPIN_BUTTON(gtk_builder_get_object(&amp;thisBuilder, "spinbuttonDown1_xyz_angle_axis_<xsl:value-of select="$name" />"));
        GtkSpinButton * spin<xsl:value-of select="$i" />_xyz_angle_axis_<xsl:value-of select="$name" /> = GTK_SPIN_BUTTON(gtk_builder_get_object(&amp;thisBuilder, "spinbutton<xsl:value-of select="$i" />_xyz_angle_axis_<xsl:value-of select="$name" />"));
        gtk_spin_button_set_value(spin<xsl:value-of select="$i" />_xyz_angle_axis_<xsl:value-of select="$name" />, gtk_spin_button_get_value(spin<xsl:value-of select="$i" />_xyz_angle_axis_<xsl:value-of select="$name" />) + gtk_spin_button_get_value(spinbuttonDown1_xyz_angle_axis_<xsl:value-of select="$name" />));
        
		on_execute_button_clicked_<xsl:value-of select="$fullName" />_xyz_angle_axis (button, userdata);
	}   

 		</xsl:when>
 		<xsl:otherwise>

	void on_button<xsl:value-of select="($i*2)-1" />_clicked_<xsl:value-of select="$fullName" />_xyz_angle_axis (GtkButton* button, gpointer userdata)
	{
		on_execute_button_clicked_<xsl:value-of select="$fullName" />_xyz_angle_axis (button, userdata);
	}
	
	void on_button<xsl:value-of select="($i*2)" />_clicked_<xsl:value-of select="$fullName" />_xyz_angle_axis (GtkButton* button, gpointer userdata)
	{
		on_execute_button_clicked_<xsl:value-of select="$fullName" />_xyz_angle_axis (button, userdata);
	}  

 		</xsl:otherwise>		
 	</xsl:choose>
    </xsl:if>
	<!-- for loop --> 
       <xsl:if test="$i &lt;= $xyz_angle_axis">
          <xsl:call-template name="for.each.edp.irp6.xyz_angle_axis.signals.cc">
              <xsl:with-param name="i">
                  <xsl:value-of select="$i + 1"/>
              </xsl:with-param>
              <xsl:with-param name="xyz_angle_axis">
                  <xsl:value-of select="$xyz_angle_axis"/>
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
