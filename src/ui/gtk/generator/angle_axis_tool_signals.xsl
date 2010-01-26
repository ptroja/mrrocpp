<?xml version="1.0" encoding="UTF-8"?>
<!--
MRROC++ GUI generator
Axis TS window callback signals
 -->
<xsl:stylesheet version="1.0" xmlns:xsl="http://www.w3.org/1999/XSL/Transform">
<xsl:output method="text"/>


<!-- signals handling file .cc-->
<xsl:template name="irp6.xyz_angle_axis_tool.main.signals.cc" match="*[substring(name(),1,4)='irp6']">
<xsl:variable name="name" select="name"/>
<xsl:variable name="fullName" select="fullName"/>
<xsl:variable name="xyz_angle_axis_tool" select="xyz_angle_axis_tool"/>

static lib::Xyz_Angle_Axis_vector tool_vector_a;

extern "C"
{
	void on_arrow_button_clicked_<xsl:value-of select="$fullName" />_xyz_angle_axis_tool (GtkButton* button, gpointer userdata)
	{
		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder &amp; thisBuilder = ((*ChoseEntry).getBuilder());
        
		<xsl:call-template name="irp6.xyz_angle_axis_tool.repeat.signals.cc.arrow">
    		<xsl:with-param name="xyz_angle_axis_tool" select="$xyz_angle_axis_tool"/>
			<xsl:with-param name="i" select="1"/>
			<xsl:with-param name="name" select="$name"/>
 		</xsl:call-template>
 	}
	
	void on_read_button_clicked_<xsl:value-of select="$fullName" />_xyz_angle_axis_tool (GtkButton* button, gpointer userdata)
	{
		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder &amp; thisBuilder = ((*ChoseEntry).getBuilder());
        
	<xsl:call-template name="irp6.xyz_angle_axis_tool.repeat.signals.cc.read.1">
    		<xsl:with-param name="xyz_angle_axis_tool" select="$xyz_angle_axis_tool"/>
			<xsl:with-param name="i" select="1"/>
			<xsl:with-param name="name" select="$name"/>
 		</xsl:call-template>
 		
		if (robot_<xsl:value-of select="$fullName" />->ecp->get_EDP_pid()!=-1)
		{
			if (state_<xsl:value-of select="$fullName" />.is_synchronised) // Czy robot jest zsynchronizowany?
			{
			    try {
    				robot_<xsl:value-of select="$fullName" />->read_tool_xyz_angle_axis(tool_vector_a); // Odczyt polozenia walow silnikow
    					
    				alfa = sqrt(tool_vector_a[3]*tool_vector_a[3]
    				            +tool_vector_a[4]*tool_vector_a[4]
    				            +tool_vector_a[5]*tool_vector_a[5]);
    				
    				if (alfa==0){
    					tool_vector_a[3] = -1;
    					tool_vector_a[4] = 0;
    					tool_vector_a[5] = 0;
    				} else {
    					tool_vector_a[3] = tool_vector_a[3]/alfa;
    					tool_vector_a[4] = tool_vector_a[4]/alfa;
    					tool_vector_a[5] = tool_vector_a[5]/alfa;
    				}
    					
    <xsl:call-template name="irp6.xyz_angle_axis_tool.repeat.signals.cc.read.2">
        				<xsl:with-param name="xyz_angle_axis_tool" select="$xyz_angle_axis_tool"/>
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
	
	void on_set_button_clicked_<xsl:value-of select="$fullName" />_xyz_angle_axis_tool (GtkButton* button, gpointer userdata)
		{
		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder &amp; thisBuilder = ((*ChoseEntry).getBuilder());
        
	<xsl:call-template name="irp6.xyz_angle_axis_tool.repeat.signals.cc.set.1">
    		<xsl:with-param name="xyz_angle_axis_tool" select="$xyz_angle_axis_tool"/>
			<xsl:with-param name="i" select="1"/>
			<xsl:with-param name="name" select="$name"/>
 		</xsl:call-template>    

		if (state_<xsl:value-of select="$fullName" />.is_synchronised)
		{
			<xsl:call-template name="irp6.xyz_angle_axis_tool.repeat.signals.cc.set.2">
   		 		<xsl:with-param name="xyz_angle_axis_tool" select="$xyz_angle_axis_tool"/>
				<xsl:with-param name="name" select="$name"/>
				<xsl:with-param name="i" select="1"/>
 			</xsl:call-template>    
 		
		 		const double wl = sqrt(tool_vector_a[3]*tool_vector_a[3] + tool_vector_a[4]*tool_vector_a[4] + tool_vector_a[5]*tool_vector_a[5]);
		
				if((wl &gt; 1 + l_eps) || (wl &lt; 1 - l_eps))
				{
					tool_vector_a[3] = tool_vector_a[3]/wl;
					tool_vector_a[4] = tool_vector_a[4]/wl;
					tool_vector_a[5] = tool_vector_a[5]/wl;
				}
				
				for(int i=3; i&lt;<xsl:value-of select="$xyz_angle_axis_tool" />; i++)
				{
						tool_vector_a[i] *= tool_vector_a[6];
				}
				try {
					robot_<xsl:value-of select="$fullName" />->set_tool_xyz_angle_axis(tool_vector_a);		
		        }
		        <xsl:call-template name="catch" />
		}
			
	    on_read_button_clicked_<xsl:value-of select="$fullName" />_xyz_angle_axis_tool (button, userdata);
	}

}

</xsl:template>

<!-- irp6 servo algorithm repeatable part -->
<xsl:template name="irp6.xyz_angle_axis_tool.repeat.signals.cc.set.1">
<xsl:param name="xyz_angle_axis_tool"/>
<xsl:param name="i"/>
<xsl:param name="name"/>
	<xsl:if test="$i &lt;= $xyz_angle_axis_tool">
		GtkSpinButton * spin<xsl:value-of select="$i" />_xyz_angle_axis_tool_<xsl:value-of select="$name" /> = GTK_SPIN_BUTTON(gtk_builder_get_object(&amp;thisBuilder, "spinbutton<xsl:value-of select="$i" />_xyz_angle_axis_tool_<xsl:value-of select="$name" />"));
 	
       </xsl:if>
	<!-- for loop --> 
       <xsl:if test="$i &lt;= $xyz_angle_axis_tool">
          <xsl:call-template name="irp6.xyz_angle_axis_tool.repeat.signals.cc.set.1">
              <xsl:with-param name="i">
                  <xsl:value-of select="$i + 1"/>
              </xsl:with-param>
              <xsl:with-param name="xyz_angle_axis_tool">
                  <xsl:value-of select="$xyz_angle_axis_tool"/>
              </xsl:with-param>
              <xsl:with-param name="name">
                  <xsl:value-of select="$name"/>
              </xsl:with-param>
          </xsl:call-template>
       </xsl:if>
</xsl:template>

<!-- irp6 servo algorithm repeatable part -->
<xsl:template name="irp6.xyz_angle_axis_tool.repeat.signals.cc.read.1">
<xsl:param name="xyz_angle_axis_tool"/>
<xsl:param name="i"/>
<xsl:param name="name"/>
	<xsl:if test="$i &lt;= $xyz_angle_axis_tool">
		GtkEntry * entry<xsl:value-of select="$i" />_xyz_angle_axis_tool_<xsl:value-of select="$name" /> = GTK_ENTRY(gtk_builder_get_object(&amp;thisBuilder, "entry<xsl:value-of select="$i" />_xyz_angle_axis_tool_<xsl:value-of select="$name" />"));
	
       </xsl:if>
	<!-- for loop --> 
       <xsl:if test="$i &lt;= $xyz_angle_axis_tool">
          <xsl:call-template name="irp6.xyz_angle_axis_tool.repeat.signals.cc.read.1">
              <xsl:with-param name="i">
                  <xsl:value-of select="$i + 1"/>
              </xsl:with-param>
              <xsl:with-param name="xyz_angle_axis_tool">
                  <xsl:value-of select="$xyz_angle_axis_tool"/>
              </xsl:with-param>
              <xsl:with-param name="name">
                  <xsl:value-of select="$name"/>
              </xsl:with-param>
          </xsl:call-template>
       </xsl:if>
</xsl:template>

<!-- irp6 servo algorithm repeatable part -->
<xsl:template name="irp6.xyz_angle_axis_tool.repeat.signals.cc.read.2">
<xsl:param name="xyz_angle_axis_tool"/>
<xsl:param name="name"/>
<xsl:param name="i"/>
	<xsl:if test="$i &lt;= $xyz_angle_axis_tool">
	<xsl:choose>
		<xsl:when test="$i &lt;= 3">
						snprintf (buf, sizeof(buf), "%.3f", tool_vector_a[<xsl:value-of select="($i - 1)" />]);
					gtk_entry_set_text(entry<xsl:value-of select="$i" />_xyz_angle_axis_tool_<xsl:value-of select="$name" />, buf);

 		</xsl:when>
 		<xsl:when test="$i &gt;= 8">
						snprintf (buf, sizeof(buf), "%.3f", tool_vector_a[<xsl:value-of select="($i - 1)" />]);
					gtk_entry_set_text(entry<xsl:value-of select="$i" />_xyz_angle_axis_tool_<xsl:value-of select="$name" />, buf);

 		</xsl:when>
<xsl:when test="$i = 7">
					snprintf (buf, sizeof(buf), "%.3f", alfa);
					gtk_entry_set_text(entry<xsl:value-of select="$i" />_xyz_angle_axis_tool_<xsl:value-of select="$name" />, buf);

 		</xsl:when>
 		<xsl:otherwise>
					snprintf (buf, sizeof(buf), "%.3f", tool_vector_a[<xsl:value-of select="($i - 1)" />]);
					gtk_entry_set_text(entry<xsl:value-of select="$i" />_xyz_angle_axis_tool_<xsl:value-of select="$name" />, buf);

 		</xsl:otherwise>		
 	</xsl:choose>
	
       </xsl:if>
	<!-- for loop --> 
       <xsl:if test="$i &lt;= $xyz_angle_axis_tool">
          <xsl:call-template name="irp6.xyz_angle_axis_tool.repeat.signals.cc.read.2">
              <xsl:with-param name="i">
                  <xsl:value-of select="$i + 1"/>
              </xsl:with-param>
              <xsl:with-param name="xyz_angle_axis_tool">
                  <xsl:value-of select="$xyz_angle_axis_tool"/>
              </xsl:with-param>
              <xsl:with-param name="name">
                  <xsl:value-of select="$name"/>
              </xsl:with-param>
          </xsl:call-template>
       </xsl:if>
</xsl:template>

<!-- irp6 servo algorithm repeatable part -->
<xsl:template name="irp6.xyz_angle_axis_tool.repeat.signals.cc.set.2">
<xsl:param name="xyz_angle_axis_tool"/>
<xsl:param name="name"/>
<xsl:param name="i"/>
	<xsl:if test="$i &lt;= $xyz_angle_axis_tool">
				tool_vector_a[<xsl:value-of select="($i - 1)" />] = gtk_spin_button_get_value(spin<xsl:value-of select="$i" />_xyz_angle_axis_tool_<xsl:value-of select="$name" />);
	
       </xsl:if>
	<!-- for loop --> 
       <xsl:if test="$i &lt;= $xyz_angle_axis_tool">
          <xsl:call-template name="irp6.xyz_angle_axis_tool.repeat.signals.cc.set.2">
              <xsl:with-param name="i">
                  <xsl:value-of select="$i + 1"/>
              </xsl:with-param>
              <xsl:with-param name="xyz_angle_axis_tool">
                  <xsl:value-of select="$xyz_angle_axis_tool"/>
              </xsl:with-param>
              <xsl:with-param name="name">
                  <xsl:value-of select="$name"/>
              </xsl:with-param>
          </xsl:call-template>
       </xsl:if>
</xsl:template>


<!-- irp6 axis ts handling signals .cc repeatable part -->
<xsl:template name="irp6.xyz_angle_axis_tool.repeat.signals.cc.arrow">
<xsl:param name="xyz_angle_axis_tool"/>
<xsl:param name="i"/>
<xsl:param name="name"/>
	<xsl:if test="$i &lt;= $xyz_angle_axis_tool">
	
        GtkEntry * entry<xsl:value-of select="$i" />_xyz_angle_axis_tool_<xsl:value-of select="$name" /> = GTK_ENTRY(gtk_builder_get_object(&amp;thisBuilder, "entry<xsl:value-of select="$i" />_xyz_angle_axis_tool_<xsl:value-of select="$name" />"));
        GtkSpinButton * spin<xsl:value-of select="$i" />_xyz_angle_axis_tool_<xsl:value-of select="$name" /> = GTK_SPIN_BUTTON(gtk_builder_get_object(&amp;thisBuilder, "spinbutton<xsl:value-of select="$i" />_xyz_angle_axis_tool_<xsl:value-of select="$name" />"));
        gtk_spin_button_set_value(spin<xsl:value-of select="$i" />_xyz_angle_axis_tool_<xsl:value-of select="$name" />, atof(gtk_entry_get_text(entry<xsl:value-of select="$i" />_xyz_angle_axis_tool_<xsl:value-of select="$name" />)));
	
       </xsl:if>
	<!-- for loop --> 
       <xsl:if test="$i &lt;= $xyz_angle_axis_tool">
          <xsl:call-template name="irp6.xyz_angle_axis_tool.repeat.signals.cc.arrow">
              <xsl:with-param name="i">
                  <xsl:value-of select="$i + 1"/>
              </xsl:with-param>
              <xsl:with-param name="xyz_angle_axis_tool">
                  <xsl:value-of select="$xyz_angle_axis_tool"/>
              </xsl:with-param>
              <xsl:with-param name="name">
                  <xsl:value-of select="$name"/>
              </xsl:with-param>
          </xsl:call-template>
       </xsl:if>
</xsl:template>

</xsl:stylesheet>
