<?xml version="1.0" encoding="UTF-8"?>
<!--
MRROC++ GUI generator
Axis TS window callback signals
 -->
<xsl:stylesheet version="1.0" xmlns:xsl="http://www.w3.org/1999/XSL/Transform">
<xsl:output method="text"/>


<!-- signals handling file .cc-->
<xsl:template name="irp6.axis.ts.main.signals.cc" match="*[substring(name(),1,4)='irp6']">
<xsl:variable name="name" select="name"/>
<xsl:variable name="fullName" select="fullName"/>
<xsl:variable name="axis_ts" select="axis_ts"/>

<xsl:text>

extern "C"
{
	void on_arrow_button_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_axis_ts (GtkButton* button, gpointer userdata)
	{
		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder &amp; thisBuilder = ((*ChoseEntry).getBuilder());
        
		</xsl:text><xsl:call-template name="irp6.axis.ts.repeat.signals.cc">
    		<xsl:with-param name="axis_ts" select="$axis_ts"/>
			<xsl:with-param name="i" select="1"/>
			<xsl:with-param name="name" select="$name"/>
 		</xsl:call-template><xsl:text>
 	}
	
	void on_read_button_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_axis_ts (GtkButton* button, gpointer userdata)
	{
		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder &amp; thisBuilder = ((*ChoseEntry).getBuilder());
        
	</xsl:text><xsl:call-template name="irp6.axis.ts.repeat.signals.cc.6">
    		<xsl:with-param name="axis_ts" select="$axis_ts"/>
			<xsl:with-param name="i" select="1"/>
			<xsl:with-param name="name" select="$name"/>
 		</xsl:call-template><xsl:text>
 		
		if (robot_</xsl:text><xsl:value-of select="$fullName" /><xsl:text></xsl:text><xsl:choose><xsl:when test="$name = 'conveyor'"></xsl:when><xsl:otherwise><xsl:text>->ecp</xsl:text></xsl:otherwise></xsl:choose><xsl:text>->get_EDP_pid()!=-1)
		{
			if (state_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>.is_synchronised) // Czy robot jest zsynchronizowany?
			{
				if (!( robot_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>->read_tool_xyz_angle_axis(tool_vector_a))) // Odczyt polozenia walow silnikow
					printf("Blad w read external\n");
					
				alfa = sqrt(tool_vector_a[3]*tool_vector_a[3]
				+tool_vector_a[4]*tool_vector_a[4]
				+tool_vector_a[5]*tool_vector_a[5]);
				
				if (alfa==0){
					tool_vector_a[3] = -1;
					tool_vector_a[4] = 0;
					tool_vector_a[5] = 0;
				}
				else{
					tool_vector_a[3] = tool_vector_a[3]/alfa;
					tool_vector_a[4] = tool_vector_a[4]/alfa;
					tool_vector_a[5] = tool_vector_a[5]/alfa;
				}
					
</xsl:text><xsl:call-template name="irp6.axis.ts.repeat.signals.cc.7">
    				<xsl:with-param name="axis_ts" select="$axis_ts"/>
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
	
	void on_set_button_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_axis_ts (GtkButton* button, gpointer userdata)
		{
		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder &amp; thisBuilder = ((*ChoseEntry).getBuilder());
        
	</xsl:text><xsl:call-template name="irp6.axis.ts.repeat.signals.cc.3">
    		<xsl:with-param name="axis_ts" select="$axis_ts"/>
			<xsl:with-param name="i" select="1"/>
			<xsl:with-param name="name" select="$name"/>
 		</xsl:call-template><xsl:text>    

		if (state_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>.is_synchronised)
		{
	</xsl:text><xsl:call-template name="irp6.axis.ts.repeat.signals.cc.8">
    		<xsl:with-param name="axis_ts" select="$axis_ts"/>
			<xsl:with-param name="name" select="$name"/>
			<xsl:with-param name="i" select="1"/>
 		</xsl:call-template><xsl:text>    
 		
 		wl = sqrt(tool_vector_a[3]*tool_vector_a[3] + tool_vector_a[4]*tool_vector_a[4] + tool_vector_a[5]*tool_vector_a[5]);

		if((wl &gt; 1 + l_eps) || (wl &lt; 1 - l_eps))
		{
			tool_vector_a[3] = tool_vector_a[3]/wl;
			tool_vector_a[4] = tool_vector_a[4]/wl;
			tool_vector_a[5] = tool_vector_a[5]/wl;
		}
		
		for(int i=3; i&lt;</xsl:text><xsl:value-of select="$axis_ts" /><xsl:text>; i++)
		{
				tool_vector_a[i] *= tool_vector_a[6];
		}
		
			robot_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>->set_tool_xyz_angle_axis(tool_vector_a);		
		}
		on_read_button_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_axis_ts (button, userdata);

	}

}
</xsl:text>
</xsl:template>

<!-- irp6 servo algorithm repeatable part -->
<xsl:template name="irp6.axis.ts.repeat.signals.cc.3">
<xsl:param name="axis_ts"/>
<xsl:param name="i"/>
<xsl:param name="name"/>
	<xsl:if test="$i &lt;= $axis_ts">
	<xsl:text>	GtkSpinButton * spin</xsl:text><xsl:value-of select="$i" /><xsl:text>_axis_ts_</xsl:text><xsl:value-of select="$name" /><xsl:text> = GTK_SPIN_BUTTON(gtk_builder_get_object(&amp;thisBuilder, "spinbutton</xsl:text><xsl:value-of select="$i" /><xsl:text>_axis_ts_</xsl:text><xsl:value-of select="$name" /><xsl:text>"));
 	</xsl:text>
       </xsl:if>
	<!-- for loop --> 
       <xsl:if test="$i &lt;= $axis_ts">
          <xsl:call-template name="irp6.axis.ts.repeat.signals.cc.3">
              <xsl:with-param name="i">
                  <xsl:value-of select="$i + 1"/>
              </xsl:with-param>
              <xsl:with-param name="axis_ts">
                  <xsl:value-of select="$axis_ts"/>
              </xsl:with-param>
              <xsl:with-param name="name">
                  <xsl:value-of select="$name"/>
              </xsl:with-param>
          </xsl:call-template>
       </xsl:if>
</xsl:template>

<!-- irp6 servo algorithm repeatable part -->
<xsl:template name="irp6.axis.ts.repeat.signals.cc.6">
<xsl:param name="axis_ts"/>
<xsl:param name="i"/>
<xsl:param name="name"/>
	<xsl:if test="$i &lt;= $axis_ts">
	<xsl:text>	GtkEntry * entry</xsl:text><xsl:value-of select="$i" /><xsl:text>_axis_ts_</xsl:text><xsl:value-of select="$name" /><xsl:text> = GTK_ENTRY(gtk_builder_get_object(&amp;thisBuilder, "entry</xsl:text><xsl:value-of select="$i" /><xsl:text>_axis_ts_</xsl:text><xsl:value-of select="$name" /><xsl:text>"));
	</xsl:text>
       </xsl:if>
	<!-- for loop --> 
       <xsl:if test="$i &lt;= $axis_ts">
          <xsl:call-template name="irp6.axis.ts.repeat.signals.cc.6">
              <xsl:with-param name="i">
                  <xsl:value-of select="$i + 1"/>
              </xsl:with-param>
              <xsl:with-param name="axis_ts">
                  <xsl:value-of select="$axis_ts"/>
              </xsl:with-param>
              <xsl:with-param name="name">
                  <xsl:value-of select="$name"/>
              </xsl:with-param>
          </xsl:call-template>
       </xsl:if>
</xsl:template>

<!-- irp6 servo algorithm repeatable part -->
<xsl:template name="irp6.axis.ts.repeat.signals.cc.7">
<xsl:param name="axis_ts"/>
<xsl:param name="name"/>
<xsl:param name="i"/>
	<xsl:if test="$i &lt;= $axis_ts">
	<xsl:choose>
		<xsl:when test="$i &lt;= 3">
	<xsl:text>					snprintf (buf, sizeof(buf), "%.3f", tool_vector_a[</xsl:text><xsl:value-of select="($i - 1)" /><xsl:text>]);
					gtk_entry_set_text(entry</xsl:text><xsl:value-of select="$i" /><xsl:text>_axis_ts_</xsl:text><xsl:value-of select="$name" /><xsl:text>, buf);
</xsl:text>
 		</xsl:when>
 		<xsl:when test="$i &gt;= 8">
	<xsl:text>					snprintf (buf, sizeof(buf), "%.3f", tool_vector_a[</xsl:text><xsl:value-of select="($i - 1)" /><xsl:text>]);
					gtk_entry_set_text(entry</xsl:text><xsl:value-of select="$i" /><xsl:text>_axis_ts_</xsl:text><xsl:value-of select="$name" /><xsl:text>, buf);
</xsl:text>
 		</xsl:when>
<xsl:when test="$i = 7">
<xsl:text>					snprintf (buf, sizeof(buf), "%.3f", alfa);
					gtk_entry_set_text(entry</xsl:text><xsl:value-of select="$i" /><xsl:text>_axis_ts_</xsl:text><xsl:value-of select="$name" /><xsl:text>, buf);
</xsl:text>
 		</xsl:when>
 		<xsl:otherwise>
<xsl:text>					snprintf (buf, sizeof(buf), "%.3f", tool_vector_a[</xsl:text><xsl:value-of select="($i - 1)" /><xsl:text>]);
					gtk_entry_set_text(entry</xsl:text><xsl:value-of select="$i" /><xsl:text>_axis_ts_</xsl:text><xsl:value-of select="$name" /><xsl:text>, buf);
</xsl:text>
 		</xsl:otherwise>		
 	</xsl:choose>
	
       </xsl:if>
	<!-- for loop --> 
       <xsl:if test="$i &lt;= $axis_ts">
          <xsl:call-template name="irp6.axis.ts.repeat.signals.cc.7">
              <xsl:with-param name="i">
                  <xsl:value-of select="$i + 1"/>
              </xsl:with-param>
              <xsl:with-param name="axis_ts">
                  <xsl:value-of select="$axis_ts"/>
              </xsl:with-param>
              <xsl:with-param name="name">
                  <xsl:value-of select="$name"/>
              </xsl:with-param>
          </xsl:call-template>
       </xsl:if>
</xsl:template>

<!-- irp6 servo algorithm repeatable part -->
<xsl:template name="irp6.axis.ts.repeat.signals.cc.8">
<xsl:param name="axis_ts"/>
<xsl:param name="name"/>
<xsl:param name="i"/>
	<xsl:if test="$i &lt;= $axis_ts">
	<xsl:text>			tool_vector_a[</xsl:text><xsl:value-of select="($i - 1)" /><xsl:text>] = gtk_spin_button_get_value(spin</xsl:text><xsl:value-of select="$i" /><xsl:text>_axis_ts_</xsl:text><xsl:value-of select="$name" /><xsl:text>);
	</xsl:text>
       </xsl:if>
	<!-- for loop --> 
       <xsl:if test="$i &lt;= $axis_ts">
          <xsl:call-template name="irp6.axis.ts.repeat.signals.cc.8">
              <xsl:with-param name="i">
                  <xsl:value-of select="$i + 1"/>
              </xsl:with-param>
              <xsl:with-param name="axis_ts">
                  <xsl:value-of select="$axis_ts"/>
              </xsl:with-param>
              <xsl:with-param name="name">
                  <xsl:value-of select="$name"/>
              </xsl:with-param>
          </xsl:call-template>
       </xsl:if>
</xsl:template>


<!-- irp6 axis ts handling signals .cc repeatable part -->
<xsl:template name="irp6.axis.ts.repeat.signals.cc">
<xsl:param name="axis_ts"/>
<xsl:param name="i"/>
<xsl:param name="name"/>
	<xsl:if test="$i &lt;= $axis_ts">
	<xsl:text>
        GtkEntry * entry</xsl:text><xsl:value-of select="$i" /><xsl:text>_axis_ts_</xsl:text><xsl:value-of select="$name" /><xsl:text> = GTK_ENTRY(gtk_builder_get_object(&amp;thisBuilder, "entry</xsl:text><xsl:value-of select="$i" /><xsl:text>_axis_ts_</xsl:text><xsl:value-of select="$name" /><xsl:text>"));
        GtkSpinButton * spin</xsl:text><xsl:value-of select="$i" /><xsl:text>_axis_ts_</xsl:text><xsl:value-of select="$name" /><xsl:text> = GTK_SPIN_BUTTON(gtk_builder_get_object(&amp;thisBuilder, "spinbutton</xsl:text><xsl:value-of select="$i" /><xsl:text>_axis_ts_</xsl:text><xsl:value-of select="$name" /><xsl:text>"));
        gtk_spin_button_set_value(spin</xsl:text><xsl:value-of select="$i" /><xsl:text>_axis_ts_</xsl:text><xsl:value-of select="$name" /><xsl:text>, atof(gtk_entry_get_text(entry</xsl:text><xsl:value-of select="$i" /><xsl:text>_axis_ts_</xsl:text><xsl:value-of select="$name" /><xsl:text>)));
	</xsl:text>
       </xsl:if>
	<!-- for loop --> 
       <xsl:if test="$i &lt;= $axis_ts">
          <xsl:call-template name="irp6.axis.ts.repeat.signals.cc">
              <xsl:with-param name="i">
                  <xsl:value-of select="$i + 1"/>
              </xsl:with-param>
              <xsl:with-param name="axis_ts">
                  <xsl:value-of select="$axis_ts"/>
              </xsl:with-param>
              <xsl:with-param name="name">
                  <xsl:value-of select="$name"/>
              </xsl:with-param>
          </xsl:call-template>
       </xsl:if>
</xsl:template>

</xsl:stylesheet>
