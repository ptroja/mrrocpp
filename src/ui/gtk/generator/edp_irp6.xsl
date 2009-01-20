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
<xsl:variable name="fullName" select="fullName"/>
<xsl:variable name="irp6EDPNumber" select="irp6EDPNumber"/>
<xsl:variable name="axis_xyz" select="axis_xyz"/>
<xsl:variable name="axis_ts" select="axis_ts"/>
<xsl:variable name="euler_xyz" select="euler_xyz"/>
<xsl:variable name="euler_ts" select="euler_ts"/>
<xsl:document method="xml" doctype-system="glade-2.0.dtd" indent="yes" version="1.0" href="../glade/edp_{$name}.glade">
<glade-interface>
  <widget class="GtkWindow" id="window">
    <child>
      <widget class="GtkVBox" id="vbox1">
        <property name="visible">True</property>
        <child>
          <widget class="GtkComboBox" id="combobox1">
            <property name="visible">True</property>
            <property name="items" translatable="yes">1. <xsl:choose><xsl:when test="$irp6EDPNumber &gt; 0">Servo algorithm</xsl:when><xsl:otherwise></xsl:otherwise> - </xsl:choose> 
2. <xsl:choose><xsl:when test="$irp6EDPNumber &gt; 0">Internal</xsl:when><xsl:otherwise> - </xsl:otherwise></xsl:choose>
3. <xsl:choose><xsl:when test="$irp6EDPNumber &gt; 0">Increment</xsl:when><xsl:otherwise> - </xsl:otherwise></xsl:choose>
4. <xsl:choose><xsl:when test="$euler_xyz &gt; 0">XYZ Euler ZYZ</xsl:when><xsl:otherwise> - </xsl:otherwise></xsl:choose> 
5. <xsl:choose><xsl:when test="$axis_xyz &gt; 0">XYZ Angle Axis</xsl:when><xsl:otherwise> - </xsl:otherwise></xsl:choose>  
6. <xsl:choose><xsl:when test="$axis_ts &gt; 0">TS Angle Axis</xsl:when><xsl:otherwise> - </xsl:otherwise></xsl:choose>
7. <xsl:choose><xsl:when test="$euler_ts &gt; 0">TS Euler ZYZ</xsl:when><xsl:otherwise> - </xsl:otherwise></xsl:choose> 
			</property>
			<signal name="changed" handler="on_combobox1_changed_{$fullName}"/>		
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
