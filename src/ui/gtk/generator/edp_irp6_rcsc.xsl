<?xml version="1.0" encoding="UTF-8"?>
<!--
MRROC++ GUI generator
EDP IRp6 RCSC window
 -->
<xsl:stylesheet version="1.0" xmlns:xsl="http://www.w3.org/1999/XSL/Transform">
<xsl:output method="xml" doctype-system="glade-2.0.dtd" indent="yes" version="1.0"/>

<!-- main edp irp6 algorithm part -->
<xsl:template name="irp6.edp.main" match="*[substring(name(),1,4)='irp6']">
<xsl:variable name="name" select="name"/>
<xsl:document method="xml" doctype-system="glade-2.0.dtd" indent="yes" version="1.0" href="../edp_{$name}_rcsc.glade">
<glade-interface>
  <widget class="GtkWindow" id="window">
    <child>
      <widget class="GtkVBox" id="vbox1">
        <property name="visible">True</property>
        <child>
          <widget class="GtkComboBox" id="combobox1">
            <property name="visible">True</property>
            <property name="items" translatable="yes">Servo algorithm
Internal
Increment
XYZ Euler ZYZ
XYZ Angle Axis
<xsl:choose><xsl:when test="$name = 'irp6p'">TS Euler ZYZ</xsl:when><xsl:otherwise>TS Angle Axis</xsl:otherwise></xsl:choose>
			</property>
			<xsl:choose>
			<xsl:when test="$name = 'irp6m'">
				<signal name="changed" handler="on_combobox1_changed_mechatronika"/>		
			</xsl:when>
			<xsl:when test="$name = 'irp6o'">
				<signal name="changed" handler="on_combobox1_changed_ontrack"/>		
			</xsl:when>
			<xsl:when test="$name = 'irp6p'">
				<signal name="changed" handler="on_combobox1_changed_postument"/>		
			</xsl:when>
		   	<xsl:otherwise>
				<signal name="changed" handler="on_combobox1_changed_newRobot"/>
    		</xsl:otherwise>	
    		</xsl:choose>
          </widget>
          <packing>
            <property name="expand">False</property>
            <property name="fill">False</property>
          </packing>
        </child>
        <child>
          <widget class="GtkScrolledWindow" id="scrolledwindow1">
            <property name="visible">True</property>
            <property name="can_focus">True</property>
            <property name="hscrollbar_policy">GTK_POLICY_AUTOMATIC</property>
            <property name="vscrollbar_policy">GTK_POLICY_AUTOMATIC</property>
            <child>
              <placeholder/>
            </child>
          </widget>
          <packing>
            <property name="position">1</property>
          </packing>
        </child>
      </widget>
    </child>
  </widget>
</glade-interface>
</xsl:document>
<xsl:call-template name="irp6.edp.main.signals.cc" />
</xsl:template>

</xsl:stylesheet>
