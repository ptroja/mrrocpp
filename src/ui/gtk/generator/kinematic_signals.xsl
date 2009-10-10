<?xml version="1.0" encoding="UTF-8"?>
<!--
MRROC++ GUI generator
kinematic window callback signals
 -->
<xsl:stylesheet version="1.0" xmlns:xsl="http://www.w3.org/1999/XSL/Transform">
<xsl:output method="text"/>

<!-- signals handling file .cc-->
<xsl:template name="irp6.kinematic.main.signals.cc" match="*[substring(name(),1,4)='irp6']">
<xsl:variable name="name" select="name"/>
<xsl:variable name="fullName" select="fullName"/>
<xsl:variable name="kinematic" select="kinematic"/>
<xsl:text>

extern "C"
{
	void on_arrow_button_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_kinematic (GtkButton* button, gpointer userdata)
	{
		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder &amp; thisBuilder = ((*ChoseEntry).getBuilder());

		</xsl:text><xsl:call-template name="irp6.kinematic.repeat.signals.cc.arrow">
    		<xsl:with-param name="kinematic" select="$kinematic"/>
			<xsl:with-param name="i" select="1"/>
			<xsl:with-param name="name" select="$name"/>
 		</xsl:call-template><xsl:text>
 	}

	void on_read_button_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_kinematic (GtkButton* button, gpointer userdata)
	{
		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder &amp; thisBuilder = ((*ChoseEntry).getBuilder());

		GtkEntry * entry1_kinematic_</xsl:text><xsl:value-of select="$name" /><xsl:text> = GTK_ENTRY(gtk_builder_get_object(&amp;thisBuilder, "entry1_kinematic_</xsl:text><xsl:value-of select="$name" /><xsl:text>"));


		if (robot_</xsl:text><xsl:value-of select="$fullName" /><xsl:text></xsl:text><xsl:choose><xsl:when test="$name = 'conveyor'"></xsl:when><xsl:otherwise><xsl:text>->ecp</xsl:text></xsl:otherwise></xsl:choose><xsl:text>->get_EDP_pid()!=-1)
		{
			if (state_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>.is_synchronised) // Czy robot jest zsynchronizowany?
			{
			    try {
    				robot_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>->get_kinematic(&amp;model_no); // Odczyt polozenia walow silnikow
    
    				snprintf (buf, sizeof(buf), "%u", model_no);
    				gtk_entry_set_text(entry1_kinematic_</xsl:text><xsl:value-of select="$name" /><xsl:text>, buf);
                }
                
                </xsl:text><xsl:call-template name="catch" /><xsl:text>
			}
			else
			{
				// Wygaszanie elementow przy niezsynchronizowanym robocie
				std::cout &lt;&lt; "Robot is not synchronized" &lt;&lt; std::endl;
			}
		}

	}

	void on_set_button_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_kinematic (GtkButton* button, gpointer userdata)
	{
		ui_config_entry * ChoseEntry = (ui_config_entry *) userdata;
        GtkBuilder &amp; thisBuilder = ((*ChoseEntry).getBuilder());

 		GtkSpinButton * spin1_kinematic_</xsl:text><xsl:value-of select="$name" /><xsl:text> = GTK_SPIN_BUTTON(gtk_builder_get_object(&amp;thisBuilder, "spinbutton1_kinematic_</xsl:text><xsl:value-of select="$name" /><xsl:text>"));

		if (state_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>.is_synchronised)
		{
			model_no_tmp = gtk_spin_button_get_value(spin1_kinematic_</xsl:text><xsl:value-of select="$name" /><xsl:text>);
			model_no = uint8_t(model_no_tmp);

			robot_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>->set_kinematic(model_no);
		}
		on_read_button_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_kinematic (button, userdata);

	}
}
</xsl:text>
</xsl:template>





<!-- irp6 axis ts handling signals .cc repeatable part -->
<xsl:template name="irp6.kinematic.repeat.signals.cc.arrow">
<xsl:param name="kinematic"/>
<xsl:param name="i"/>
<xsl:param name="name"/>
	<xsl:if test="$i &lt;= $kinematic">
	<xsl:text>
        GtkEntry * entry</xsl:text><xsl:value-of select="$i" /><xsl:text>_kinematic_</xsl:text><xsl:value-of select="$name" /><xsl:text> = GTK_ENTRY(gtk_builder_get_object(&amp;thisBuilder, "entry</xsl:text><xsl:value-of select="$i" /><xsl:text>_kinematic_</xsl:text><xsl:value-of select="$name" /><xsl:text>"));
        GtkSpinButton * spin</xsl:text><xsl:value-of select="$i" /><xsl:text>_kinematic_</xsl:text><xsl:value-of select="$name" /><xsl:text> = GTK_SPIN_BUTTON(gtk_builder_get_object(&amp;thisBuilder, "spinbutton</xsl:text><xsl:value-of select="$i" /><xsl:text>_kinematic_</xsl:text><xsl:value-of select="$name" /><xsl:text>"));
        gtk_spin_button_set_value(spin</xsl:text><xsl:value-of select="$i" /><xsl:text>_kinematic_</xsl:text><xsl:value-of select="$name" /><xsl:text>, atof(gtk_entry_get_text(entry</xsl:text><xsl:value-of select="$i" /><xsl:text>_kinematic_</xsl:text><xsl:value-of select="$name" /><xsl:text>)));
	</xsl:text>
       </xsl:if>
	<!-- for loop -->
       <xsl:if test="$i &lt;= $kinematic">
          <xsl:call-template name="irp6.kinematic.repeat.signals.cc.arrow">
              <xsl:with-param name="i">
                  <xsl:value-of select="$i + 1"/>
              </xsl:with-param>
              <xsl:with-param name="kinematic">
                  <xsl:value-of select="$kinematic"/>
              </xsl:with-param>
              <xsl:with-param name="name">
                  <xsl:value-of select="$name"/>
              </xsl:with-param>
          </xsl:call-template>
       </xsl:if>
</xsl:template>

</xsl:stylesheet>
