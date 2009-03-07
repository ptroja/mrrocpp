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
<xsl:variable name="motorsNo" select="motorsNo"/>
<xsl:text>

extern "C"
{
	void on_arrow_button_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_servo (GtkButton* button, gpointer userdata)
	{
		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder &amp; thisBuilder = ((*ChoseEntry).getBuilder());
        
		</xsl:text><xsl:call-template name="irp6.servo.repeat.signals.cc">
    		<xsl:with-param name="motorsNo" select="$motorsNo"/>
			<xsl:with-param name="i" select="1"/>
			<xsl:with-param name="name" select="$name"/>
 		</xsl:call-template><xsl:text>
	}
	
	void on_read_button_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_servo (GtkButton* button, gpointer userdata)
	{
		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder &amp; thisBuilder = ((*ChoseEntry).getBuilder());
        
</xsl:text><xsl:call-template name="irp6.servo.repeat.signals.cc.1">
    		<xsl:with-param name="motorsNo" select="$motorsNo"/>
			<xsl:with-param name="i" select="1"/>
			<xsl:with-param name="name" select="$name"/>
 		</xsl:call-template><xsl:text>

		if (robot_</xsl:text><xsl:value-of select="$fullName" /><xsl:text></xsl:text><xsl:choose><xsl:when test="$name = 'conveyor'"></xsl:when><xsl:otherwise><xsl:text>->ecp</xsl:text></xsl:otherwise></xsl:choose><xsl:text>->get_EDP_pid()!=-1)
		{
				if (state.is_synchronised)  // Czy robot jest zsynchronizowany?
				{
					if (!(robot_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>->get_servo_algorithm(servo_alg_no, servo_par_no))) // Odczyt polozenia walow silnikow
						printf("Blad w mechatronika get_servo_algorithm\n");
					
</xsl:text><xsl:call-template name="irp6.servo.repeat.signals.cc.2">
    						<xsl:with-param name="motorsNo" select="$motorsNo"/>
							<xsl:with-param name="i" select="1"/>
							<xsl:with-param name="name" select="$name"/>
 						</xsl:call-template><xsl:text>					
				} else
				{
					std::cout &lt;&lt; "Robot is not synchronized" &lt;&lt; std::endl;
				}
			}
	}
	
	void on_set_button_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_servo (GtkButton* button, gpointer userdata)
	{
		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder &amp; thisBuilder = ((*ChoseEntry).getBuilder());

</xsl:text><xsl:call-template name="irp6.servo.repeat.signals.cc.3">
    		<xsl:with-param name="motorsNo" select="$motorsNo"/>
			<xsl:with-param name="i" select="1"/>
			<xsl:with-param name="name" select="$name"/>
 		</xsl:call-template><xsl:text>
 		
 		if (state.is_synchronised)
		{
</xsl:text><xsl:call-template name="irp6.servo.repeat.signals.cc.4">
    		<xsl:with-param name="motorsNo" select="$motorsNo"/>
			<xsl:with-param name="i" select="1"/>
			<xsl:with-param name="name" select="$name"/>
 		</xsl:call-template><xsl:text>

		for(int i=0; i&lt;</xsl:text><xsl:value-of select="$motorsNo" /><xsl:text>; i++)
		{
			servo_alg_no_output[i] = BYTE(servo_alg_no_tmp[i]);
			servo_par_no_output[i] = BYTE(servo_par_no_tmp[i]);
		}

		// zlecenie wykonania ruchu
		robot_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>->set_servo_algorithm(servo_alg_no_output, servo_par_no_output);

	}
	else
	{
		std::cout &lt;&lt; "Robot is not synchronized" &lt;&lt; std::endl;
	}
 		
	}
}
</xsl:text>
</xsl:template>

<!-- irp6 servo algorithm repeatable part -->
<xsl:template name="irp6.servo.repeat.signals.cc.1">
<xsl:param name="motorsNo"/>
<xsl:param name="i"/>
<xsl:param name="name"/>
	<xsl:if test="$i &lt;= $motorsNo*2">
	<xsl:text>		GtkEntry * entry</xsl:text><xsl:value-of select="$i" /><xsl:text>_servo_</xsl:text><xsl:value-of select="$name" /><xsl:text> = GTK_ENTRY(gtk_builder_get_object(&amp;thisBuilder, "entry</xsl:text><xsl:value-of select="$i" /><xsl:text>_servo_</xsl:text><xsl:value-of select="$name" /><xsl:text>"));
</xsl:text>
       </xsl:if>
	<!-- for loop --> 
       <xsl:if test="$i &lt;= $motorsNo*2">
          <xsl:call-template name="irp6.servo.repeat.signals.cc.1">
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
<xsl:template name="irp6.servo.repeat.signals.cc.2">
<xsl:param name="motorsNo"/>
<xsl:param name="i"/>
<xsl:param name="name"/>
	<xsl:if test="$i &lt;= $motorsNo">
	<xsl:text>					gtk_entry_set_text(entry</xsl:text><xsl:value-of select="($i*2)-1" /><xsl:text>_servo_</xsl:text><xsl:value-of select="$name" /><xsl:text>, (const gchar*)servo_alg_no[</xsl:text><xsl:value-of select="($i - 1)" /><xsl:text>]);
					gtk_entry_set_text(entry</xsl:text><xsl:value-of select="($i*2)" /><xsl:text>_servo_</xsl:text><xsl:value-of select="$name" /><xsl:text>, (const gchar*)servo_par_no[</xsl:text><xsl:value-of select="($i - 1)" /><xsl:text>]);	
</xsl:text>
       </xsl:if>
	<!-- for loop --> 
       <xsl:if test="$i &lt;= $motorsNo">
          <xsl:call-template name="irp6.servo.repeat.signals.cc.2">
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
<xsl:template name="irp6.servo.repeat.signals.cc.3">
<xsl:param name="motorsNo"/>
<xsl:param name="i"/>
<xsl:param name="name"/>
	<xsl:if test="$i &lt;= $motorsNo*2">
	<xsl:text>		GtkSpinButton * spin</xsl:text><xsl:value-of select="$i" /><xsl:text>_servo_</xsl:text><xsl:value-of select="$name" /><xsl:text> = GTK_SPIN_BUTTON(gtk_builder_get_object(&amp;thisBuilder, "spinbutton</xsl:text><xsl:value-of select="$i" /><xsl:text>_servo_</xsl:text><xsl:value-of select="$name" /><xsl:text>"));
</xsl:text>
       </xsl:if>
	<!-- for loop --> 
       <xsl:if test="$i &lt;= $motorsNo*2">
          <xsl:call-template name="irp6.servo.repeat.signals.cc.3">
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
<xsl:template name="irp6.servo.repeat.signals.cc.4">
<xsl:param name="motorsNo"/>
<xsl:param name="i"/>
<xsl:param name="name"/>
	<xsl:if test="$i &lt;= $motorsNo">
	<xsl:text>			servo_alg_no_tmp[</xsl:text><xsl:value-of select="($i - 1)" /><xsl:text>] = gtk_spin_button_get_value_as_int(spin</xsl:text><xsl:value-of select="($i*2)-1" /><xsl:text>_servo_</xsl:text><xsl:value-of select="$name" /><xsl:text>);
			servo_par_no_tmp[</xsl:text><xsl:value-of select="($i - 1)" /><xsl:text>] = gtk_spin_button_get_value_as_int(spin</xsl:text><xsl:value-of select="($i*2)" /><xsl:text>_servo_</xsl:text><xsl:value-of select="$name" /><xsl:text>);
</xsl:text>
       </xsl:if>
	<!-- for loop --> 
       <xsl:if test="$i &lt;= $motorsNo">
          <xsl:call-template name="irp6.servo.repeat.signals.cc.4">
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
<xsl:template name="irp6.servo.repeat.signals.cc">
<xsl:param name="motorsNo"/>
<xsl:param name="i"/>
<xsl:param name="name"/>
	<xsl:if test="$i &lt;= $motorsNo*2">
	<xsl:text>
        GtkEntry * entry</xsl:text><xsl:value-of select="$i" /><xsl:text>_servo_</xsl:text><xsl:value-of select="$name" /><xsl:text> = GTK_ENTRY(gtk_builder_get_object(&amp;thisBuilder, "entry</xsl:text><xsl:value-of select="$i" /><xsl:text>_servo_</xsl:text><xsl:value-of select="$name" /><xsl:text>"));
        GtkSpinButton * spin</xsl:text><xsl:value-of select="$i" /><xsl:text>_servo_</xsl:text><xsl:value-of select="$name" /><xsl:text> = GTK_SPIN_BUTTON(gtk_builder_get_object(&amp;thisBuilder, "spinbutton</xsl:text><xsl:value-of select="$i" /><xsl:text>_servo_</xsl:text><xsl:value-of select="$name" /><xsl:text>"));
        gtk_spin_button_set_value(spin</xsl:text><xsl:value-of select="$i" /><xsl:text>_servo_</xsl:text><xsl:value-of select="$name" /><xsl:text>, atof(gtk_entry_get_text(entry</xsl:text><xsl:value-of select="$i" /><xsl:text>_servo_</xsl:text><xsl:value-of select="$name" /><xsl:text>)));
	</xsl:text>
       </xsl:if>
	<!-- for loop --> 
       <xsl:if test="$i &lt;= $motorsNo*2">
          <xsl:call-template name="irp6.servo.repeat.signals.cc">
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

</xsl:stylesheet>
