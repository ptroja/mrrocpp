<?xml version="1.0" encoding="UTF-8"?>
<!--
MRROC++ GUI generator
Main robot window
 -->
<xsl:stylesheet version="1.0" xmlns:xsl="http://www.w3.org/1999/XSL/Transform">
<xsl:output method="xml" doctype-system="glade-2.0.dtd" indent="yes" version="1.0"/>

<!-- main edp irp6 algorithm part -->
<xsl:template name="irp6.edp.main" match="*[substring(name(),1,4)='irp6']">
<xsl:variable name="name" select="name"/>
<xsl:variable name="fullName" select="fullName"/>
<xsl:variable name="motorsNo" select="motorsNo"/>
<xsl:variable name="xyz_angle_axis" select="xyz_angle_axis"/>
<xsl:variable name="xyz_angle_axis_tool" select="xyz_angle_axis_tool"/>
<xsl:variable name="xyz_euler_zyz" select="xyz_euler_zyz"/>
<xsl:variable name="xyz_euler_zyz_tool" select="xyz_euler_zyz_tool"/>
<xsl:document method="xml" doctype-system="glade-2.0.dtd" indent="yes" version="1.0" href="../glade/edp_{$name}.glade">
<glade-interface>
  <widget class="GtkWindow" id="window">
    <child>
      <widget class="GtkVBox" id="vbox1">
        <property name="visible">True</property>
         <child>
          <widget class="GtkHBox" id="hbox1">
            <property name="visible">True</property>
            <child>
          <widget class="GtkComboBox" id="combobox1">
            <property name="visible">True</property>
            <property name="items" translatable="yes"> -</property>
			<signal name="changed" handler="on_combobox1_changed_{$fullName}"/>		
    	  </widget>
        </child>
            <child>
              <widget class="GtkButton" id="button_synchronize">
                <property name="visible">True</property>
                <property name="can_focus">True</property>
                <property name="receives_default">True</property>
                <property name="label" translatable="yes">  Synchronize  </property>
                <property name="response_id">0</property>
                <signal name="clicked" handler="on_clicked_synchronize_{$fullName}"/>	
              </widget>
              <packing>
                <property name="expand">False</property>
                <property name="pack_type">GTK_PACK_END</property>
                <property name="position">1</property>
              </packing>
            </child>
          </widget>
          <packing>
            <property name="expand">False</property>
            <property name="fill">False</property>
          </packing>
        </child>
        <child>
          <widget class="GtkScrolledWindow" id="scrolledwindow_edp">
            <property name="visible">True</property>
            <property name="can_focus">True</property>
            <property name="hscrollbar_policy">GTK_POLICY_AUTOMATIC</property>
            <property name="vscrollbar_policy">GTK_POLICY_AUTOMATIC</property>
            <child>
              <widget class="GtkViewport" id="viewport8">
                <property name="visible">True</property>
                <property name="resize_mode">GTK_RESIZE_QUEUE</property>
                <child>
                  <placeholder/>
                </child>
              </widget>
            </child>
          </widget>
          <packing>
            <property name="position">1</property>
          </packing>
        </child>
      </widget>
    </child>
  </widget>

<xsl:call-template name="irp6.xyz_angle_axis_tool" />
<xsl:call-template name="irp6.xyz_angle_axis" />
<xsl:call-template name="irp6.xyz_euler_zyz_tool" />
<xsl:call-template name="irp6.xyz_euler_zyz" />
<xsl:call-template name="irp6.inc" />
<xsl:call-template name="irp6.int" />
<xsl:call-template name="irp6.servo" />
<xsl:call-template name="irp6.kinematic" />
</glade-interface>
</xsl:document>
<xsl:call-template name="irp6.edp.main.signals.cc" />
</xsl:template>

</xsl:stylesheet>
