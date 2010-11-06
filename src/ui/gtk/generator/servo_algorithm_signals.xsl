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


extern "C"
{
	void on_arrow_button_clicked_<xsl:value-of select="$fullName" />_servo (GtkButton* button, gpointer userdata)
	{
		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder &amp; thisBuilder = ((*ChoseEntry).getBuilder());
        
		<xsl:call-template name="irp6.servo.repeat.signals.cc.arrow">
    		<xsl:with-param name="motorsNo" select="$motorsNo"/>
			<xsl:with-param name="i" select="1"/>
			<xsl:with-param name="name" select="$name"/>
 		</xsl:call-template>
	}
	
	void on_read_button_clicked_<xsl:value-of select="$fullName" />_servo (GtkButton* button, gpointer userdata)
	{
		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder &amp; thisBuilder = ((*ChoseEntry).getBuilder());
        
<xsl:call-template name="irp6.servo.repeat.signals.cc.read.1">
    		<xsl:with-param name="motorsNo" select="$motorsNo"/>
			<xsl:with-param name="i" select="1"/>
			<xsl:with-param name="name" select="$name"/>
 		</xsl:call-template>

		if (robot_<xsl:value-of select="$fullName" />->ecp->get_EDP_pid()!=-1)
		{
				if (state_<xsl:value-of select="$fullName" />.is_synchronised)  // Czy robot jest zsynchronizowany?
				{
				    try {
    					robot_<xsl:value-of select="$fullName" />->get_servo_algorithm(servo_alg_no, servo_par_no); // Odczyt polozenia walow silnikow
    					
    <xsl:call-template name="irp6.servo.repeat.signals.cc.read.2">
        						<xsl:with-param name="motorsNo" select="$motorsNo"/>
    							<xsl:with-param name="i" select="1"/>
    							<xsl:with-param name="name" select="$name"/>
     						</xsl:call-template>					
     				}
     				
     				<xsl:call-template name="catch" />
				} else
				{
					std::cout &lt;&lt; "Robot is not synchronized" &lt;&lt; std::endl;
				}
			}
	}
	
	void on_set_button_clicked_<xsl:value-of select="$fullName" />_servo (GtkButton* button, gpointer userdata)
	{
		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder &amp; thisBuilder = ((*ChoseEntry).getBuilder());

<xsl:call-template name="irp6.servo.repeat.signals.cc.set.1">
    		<xsl:with-param name="motorsNo" select="$motorsNo"/>
			<xsl:with-param name="i" select="1"/>
			<xsl:with-param name="name" select="$name"/>
 		</xsl:call-template>
 		
 		if (state_<xsl:value-of select="$fullName" />.is_synchronised)
		{
<xsl:call-template name="irp6.servo.repeat.signals.cc.set.2">
    		<xsl:with-param name="motorsNo" select="$motorsNo"/>
			<xsl:with-param name="i" select="1"/>
			<xsl:with-param name="name" select="$name"/>
 		</xsl:call-template>

		for(int i=0; i&lt;<xsl:value-of select="$motorsNo" />; i++)
		{
			servo_alg_no_output[i] = uint8_t(servo_alg_no_tmp[i]);
			servo_par_no_output[i] = uint8_t(servo_par_no_tmp[i]);
		}

		// zlecenie wykonania ruchu
		try {
			robot_<xsl:value-of select="$fullName" />->set_servo_algorithm(servo_alg_no_output, servo_par_no_output);
		}
		<xsl:call-template name="catch" />

	}
	else
	{
		std::cout &lt;&lt; "Robot is not synchronized" &lt;&lt; std::endl;
	}
 		
	}
}

</xsl:template>

<!-- irp6 servo algorithm repeatable part -->
<xsl:template name="irp6.servo.repeat.signals.cc.read.1">
<xsl:param name="motorsNo"/>
<xsl:param name="i"/>
<xsl:param name="name"/>
	<xsl:if test="$i &lt;= $motorsNo*2">
			GtkEntry * entry<xsl:value-of select="$i" />_servo_<xsl:value-of select="$name" /> = GTK_ENTRY(gtk_builder_get_object(&amp;thisBuilder, "entry<xsl:value-of select="$i" />_servo_<xsl:value-of select="$name" />"));

       </xsl:if>
	<!-- for loop --> 
       <xsl:if test="$i &lt;= $motorsNo*2">
          <xsl:call-template name="irp6.servo.repeat.signals.cc.read.1">
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
<xsl:template name="irp6.servo.repeat.signals.cc.read.2">
<xsl:param name="motorsNo"/>
<xsl:param name="i"/>
<xsl:param name="name"/>
	<xsl:if test="$i &lt;= $motorsNo">
		
					snprintf (buf, sizeof(buf), "%u", servo_alg_no[<xsl:value-of select="($i - 1)" />]);
					gtk_entry_set_text(entry<xsl:value-of select="($i*2)-1" />_servo_<xsl:value-of select="$name" />, buf);
					snprintf (buf, sizeof(buf), "%u", servo_par_no[<xsl:value-of select="($i - 1)" />]);
					gtk_entry_set_text(entry<xsl:value-of select="($i*2)" />_servo_<xsl:value-of select="$name" />, buf);	

       </xsl:if>
	<!-- for loop --> 
       <xsl:if test="$i &lt;= $motorsNo">
          <xsl:call-template name="irp6.servo.repeat.signals.cc.read.2">
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
<xsl:template name="irp6.servo.repeat.signals.cc.set.1">
<xsl:param name="motorsNo"/>
<xsl:param name="i"/>
<xsl:param name="name"/>
	<xsl:if test="$i &lt;= $motorsNo*2">
			GtkSpinButton * spin<xsl:value-of select="$i" />_servo_<xsl:value-of select="$name" /> = GTK_SPIN_BUTTON(gtk_builder_get_object(&amp;thisBuilder, "spinbutton<xsl:value-of select="$i" />_servo_<xsl:value-of select="$name" />"));

       </xsl:if>
	<!-- for loop --> 
       <xsl:if test="$i &lt;= $motorsNo*2">
          <xsl:call-template name="irp6.servo.repeat.signals.cc.set.1">
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
<xsl:template name="irp6.servo.repeat.signals.cc.set.2">
<xsl:param name="motorsNo"/>
<xsl:param name="i"/>
<xsl:param name="name"/>
	<xsl:if test="$i &lt;= $motorsNo">
				servo_alg_no_tmp[<xsl:value-of select="($i - 1)" />] = gtk_spin_button_get_value_as_int(spin<xsl:value-of select="($i*2)-1" />_servo_<xsl:value-of select="$name" />);
			servo_par_no_tmp[<xsl:value-of select="($i - 1)" />] = gtk_spin_button_get_value_as_int(spin<xsl:value-of select="($i*2)" />_servo_<xsl:value-of select="$name" />);

       </xsl:if>
	<!-- for loop --> 
       <xsl:if test="$i &lt;= $motorsNo">
          <xsl:call-template name="irp6.servo.repeat.signals.cc.set.2">
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
<xsl:template name="irp6.servo.repeat.signals.cc.arrow">
<xsl:param name="motorsNo"/>
<xsl:param name="i"/>
<xsl:param name="name"/>
	<xsl:if test="$i &lt;= $motorsNo*2">
	
        GtkEntry * entry<xsl:value-of select="$i" />_servo_<xsl:value-of select="$name" /> = GTK_ENTRY(gtk_builder_get_object(&amp;thisBuilder, "entry<xsl:value-of select="$i" />_servo_<xsl:value-of select="$name" />"));
        GtkSpinButton * spin<xsl:value-of select="$i" />_servo_<xsl:value-of select="$name" /> = GTK_SPIN_BUTTON(gtk_builder_get_object(&amp;thisBuilder, "spinbutton<xsl:value-of select="$i" />_servo_<xsl:value-of select="$name" />"));
        gtk_spin_button_set_value(spin<xsl:value-of select="$i" />_servo_<xsl:value-of select="$name" />, atof(gtk_entry_get_text(entry<xsl:value-of select="$i" />_servo_<xsl:value-of select="$name" />)));
	
       </xsl:if>
	<!-- for loop --> 
       <xsl:if test="$i &lt;= $motorsNo*2">
          <xsl:call-template name="irp6.servo.repeat.signals.cc.arrow">
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
