<?xml version="1.0" encoding="UTF-8"?>
<!--
MRROC++ GUI generator
Festival window
 -->
<xsl:stylesheet version="1.0" xmlns:xsl="http://www.w3.org/1999/XSL/Transform">
<xsl:output method="xml" doctype-system="glade-2.0.dtd" indent="yes" version="1.0"/>


<!-- main festival window -->
<xsl:template name="festival.main" match="festival">
<xsl:variable name="name" select="name"/>
<xsl:document method="xml" doctype-system="glade-2.0.dtd" indent="yes" version="1.0" href="../glade/{$name}.glade">

<glade-interface>
  <widget class="GtkWindow" id="window">
    <child>
      <widget class="GtkVBox" id="vbox1">
        <property name="visible">True</property>
        <child>
          <widget class="GtkLabel" id="label1">
            <property name="visible">True</property>
            <property name="label" translatable="yes">Festival Speech Process</property>
          </widget>
        </child>
        <child>
          <widget class="GtkComboBox" id="LanguageCombo">
            <property name="visible">True</property>
            <property name="add_tearoffs">True</property>
            <property name="items" translatable="yes">English
Polish</property>
          </widget>
          <packing>
            <property name="expand">False</property>
            <property name="fill">False</property>
            <property name="position">1</property>
          </packing>
        </child>
        <child>
          <widget class="GtkHBox" id="hbox1">
            <property name="visible">True</property>
            <child>
              <widget class="GtkEntry" id="entry1">
                <property name="visible">True</property>
                <property name="can_focus">True</property>
              </widget>
            </child>
            <child>
              <widget class="GtkHButtonBox" id="hbuttonbox1">
                <property name="visible">True</property>
                <child>
                  <widget class="GtkButton" id="button1">
                    <property name="visible">True</property>
                    <property name="can_focus">True</property>
                    <property name="receives_default">True</property>
                    <property name="label" translatable="yes">Say</property>
                    <property name="response_id">0</property>
                    <signal name="clicked" handler="on_say_button_clicked_{$name}"/>
                  </widget>
                </child>
              </widget>
              <packing>
                <property name="position">1</property>
              </packing>
            </child>
          </widget>
          <packing>
            <property name="position">2</property>
          </packing>
        </child>
      </widget>
    </child>
  </widget>
</glade-interface>

</xsl:document>	
</xsl:template>
</xsl:stylesheet>
