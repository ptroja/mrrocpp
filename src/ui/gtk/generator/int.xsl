<?xml version="1.0" encoding="UTF-8"?>
<!--
MRROC++ GUI generator
Int window
 -->
<xsl:stylesheet version="1.0" xmlns:xsl="http://www.w3.org/1999/XSL/Transform">
<xsl:output method="xml" doctype-system="glade-2.0.dtd" indent="yes" version="1.0"/>


<!-- main irp6 int part -->
<xsl:template name="irp6.int" match="*[substring(name(),1,4)='irp6']">
<xsl:variable name="name" select="name"/>
<xsl:variable name="fullName" select="fullName"/>
<xsl:variable name="motorsNo" select="motorsNo"/>
  <widget class="GtkWindow" id="window_int">
    <child>
      <widget class="GtkScrolledWindow" id="scrolledwindow1"><xsl:attribute name="id">scrolledwindow_int_<xsl:value-of select="$name" /></xsl:attribute>
        <property name="visible">True</property>
        <property name="can_focus">True</property>
        <property name="hscrollbar_policy">GTK_POLICY_AUTOMATIC</property>
        <property name="vscrollbar_policy">GTK_POLICY_AUTOMATIC</property>
        <child>
          <widget class="GtkViewport" id="viewport1"><xsl:attribute name="id">viewport_int_<xsl:value-of select="$name" /></xsl:attribute>
            <property name="visible">True</property>
            <property name="resize_mode">GTK_RESIZE_QUEUE</property>
            <child>
              <widget class="GtkTable" id="table1"><xsl:attribute name="id">table_int_<xsl:value-of select="$name" /></xsl:attribute>
                <property name="visible">True</property>
                <property name="n_rows"><xsl:value-of select="$motorsNo + 5" /></property> <!-- 5 + RN  -->
                <property name="n_columns">9</property>
                <child>
                  <placeholder/>
                </child>
                <child>
                  <placeholder/>
                </child>
                <child>
                  <placeholder/>
                </child>
                <child>
                  <placeholder/>
                </child>
                <child>
                  <placeholder/>
                </child>
                <child>
                  <placeholder/>
                </child>
                <child>
                  <placeholder/>
                </child>
                <child>
                  <placeholder/>
                </child>
                <child>
                  <placeholder/>
                </child>
                <child>
                  <widget class="GtkSpinButton" id="spinbuttonDown1"><xsl:attribute name="id">spinbuttonDown1_int_<xsl:value-of select="$name" /></xsl:attribute>
                    <property name="width_request">50</property>
                    <property name="visible">True</property>
                    <property name="can_focus">True</property>
                    <property name="adjustment">0 -999999999999999 999999999999999 0.01 0.1 0</property>
                    <property name="digits">3</property> 
                  </widget>
                  <packing>
                    <property name="left_attach">7</property>
                    <property name="right_attach">9</property>
                    <property name="top_attach"><xsl:value-of select="$motorsNo + 3" /></property>  <!-- 3 + RN  -->
                    <property name="bottom_attach"><xsl:value-of select="$motorsNo + 4" /></property>  <!-- 4 + RN  -->
                  </packing>
                </child>
                <child>
                  <widget class="GtkButton" id="buttonDown1"><xsl:attribute name="id">buttonDown1_int_<xsl:value-of select="$name" /></xsl:attribute>
                    <property name="visible">True</property>
                    <property name="can_focus">True</property>
                    <property name="receives_default">True</property>
                    <property name="label" translatable="yes">Execute motion</property>
                    <property name="response_id">0</property>
                    <signal name="clicked" handler="on_execute_button_clicked_{$fullName}_int"/>
                  </widget>
                  <packing>
                    <property name="left_attach">5</property>
                    <property name="right_attach">6</property>
                    <property name="top_attach"><xsl:value-of select="$motorsNo + 3" /></property>  <!-- 3 + RN  -->
                    <property name="bottom_attach"><xsl:value-of select="$motorsNo + 4" /></property> <!-- 4 + RN  -->
                  </packing>
                </child>
                <child>
                  <widget class="GtkButton" id="buttonDown2"><xsl:attribute name="id">buttonDown2_int_<xsl:value-of select="$name" /></xsl:attribute>
                    <property name="visible">True</property>
                    <property name="can_focus">True</property>
                    <property name="receives_default">True</property>
                    <property name="label" translatable="yes">Import</property>
                    <property name="response_id">0</property>
                    <signal name="clicked" handler="on_import_button_clicked_{$fullName}_int"/>
                  </widget>
                  <packing>
                    <property name="left_attach">2</property>
                    <property name="right_attach">3</property>
                    <property name="top_attach"><xsl:value-of select="$motorsNo + 4" /></property>  <!-- 4 + RN  -->
                    <property name="bottom_attach"><xsl:value-of select="$motorsNo + 5" /></property>  <!-- 5 + RN  -->
                  </packing>
                </child>
                <child>
                  <widget class="GtkButton" id="buttonLeft1"><xsl:attribute name="id">buttonLeft1_int_<xsl:value-of select="$name" /></xsl:attribute>
                    <property name="visible">True</property>
                    <property name="can_focus">True</property>
                    <property name="receives_default">True</property>
                    <property name="response_id">0</property>
                    <signal name="clicked" handler="on_arrow_button_clicked_{$fullName}_int"/>	
                    <child>
                      <widget class="GtkArrow" id="arrowLeft1"><xsl:attribute name="id">arrowLeft1_int_<xsl:value-of select="$name" /></xsl:attribute>
                        <property name="visible">True</property>
                      </widget>
                    </child>
                  </widget>
                  <packing>
                    <property name="left_attach">3</property>
                    <property name="right_attach">4</property>
                    <property name="top_attach">2</property>
                    <property name="bottom_attach"><xsl:value-of select="$motorsNo + 2" /></property> <!-- 2 + RN  -->
                    <property name="x_options"></property>
                    <property name="x_padding">5</property>
                  </packing>
                </child>
                <child>
                  <widget class="GtkButton" id="buttonDown3"><xsl:attribute name="id">buttonDown3_int_<xsl:value-of select="$name" /></xsl:attribute>
                    <property name="visible">True</property>
                    <property name="can_focus">True</property>
                    <property name="receives_default">True</property>
                    <property name="label" translatable="yes">Read</property>
                    <property name="response_id">0</property>
					<signal name="clicked" handler="on_read_button_clicked_{$fullName}_int"/>	
                  </widget>
                  <packing>
                    <property name="left_attach">2</property>
                    <property name="right_attach">3</property>
                    <property name="top_attach"><xsl:value-of select="$motorsNo + 3" /></property>  <!-- 3 + RN  -->
                    <property name="bottom_attach"><xsl:value-of select="$motorsNo + 4" /></property>  <!-- 4 + RN  -->
                  </packing>
                </child>
                <child>
                  <widget class="GtkButton" id="buttonDown4"><xsl:attribute name="id">buttonDown4_int_<xsl:value-of select="$name" /></xsl:attribute>
                    <property name="visible">True</property>
                    <property name="can_focus">True</property>
                    <property name="receives_default">True</property>
                    <property name="label" translatable="yes">Export</property>
                    <property name="response_id">0</property>
                    <signal name="clicked" handler="on_export_button_clicked_{$fullName}_int"/>
                  </widget>
                  <packing>
                    <property name="left_attach">5</property>
                    <property name="right_attach">6</property>
                    <property name="top_attach"><xsl:value-of select="$motorsNo + 4" /></property> <!-- 4 + RN  -->
                    <property name="bottom_attach"><xsl:value-of select="$motorsNo + 5" /></property> <!-- 5 + RN  -->
                  </packing>
                </child>
                <child>
                  <widget class="GtkLabel" id="labelDown1"><xsl:attribute name="id">labelDown1_int_<xsl:value-of select="$name" /></xsl:attribute>
                    <property name="visible">True</property>
                    <property name="label" translatable="yes">step</property>
                  </widget>
                  <packing>
                    <property name="left_attach">7</property>
                    <property name="right_attach">9</property>
                    <property name="top_attach"><xsl:value-of select="$motorsNo + 4" /></property>  <!-- 4 + RN  -->
                    <property name="bottom_attach"><xsl:value-of select="$motorsNo + 5" /></property>  <!-- 5 + RN  -->
                  </packing>
                </child>
                <child>
                  <widget class="GtkLabel" id="labelUp1"><xsl:attribute name="id">labelUp1_int_<xsl:value-of select="$name" /></xsl:attribute>
                    <property name="visible">True</property>
                    <property name="label" translatable="yes">axis</property>
                  </widget>
                </child>
                <child>
                  <widget class="GtkLabel" id="labelUp2"><xsl:attribute name="id">labelUp2_int_<xsl:value-of select="$name" /></xsl:attribute>
                    <property name="width_request">77</property>
                    <property name="visible">True</property>
                    <property name="label" translatable="yes">current position</property>
                  </widget>
                  <packing>
                    <property name="left_attach">2</property>
                    <property name="right_attach">4</property>
                  </packing>
                </child>
                <child>
                  <widget class="GtkLabel" id="labelUp3"><xsl:attribute name="id">labelUp3_int_<xsl:value-of select="$name" /></xsl:attribute>
                    <property name="visible">True</property>
                    <property name="label" translatable="yes">incremental move</property>
                  </widget>
                  <packing>
                    <property name="left_attach">7</property>
                    <property name="right_attach">9</property>
                  </packing>
                </child>
                <child>
                  <widget class="GtkLabel" id="labelUp4"><xsl:attribute name="id">labelUp4_int_<xsl:value-of select="$name" /></xsl:attribute>
                    <property name="width_request">77</property>
                    <property name="visible">True</property>
                    <property name="label" translatable="yes">desired position</property>
                  </widget>
                  <packing>
                    <property name="left_attach">5</property>
                    <property name="right_attach">6</property>
                  </packing>
                </child>
                <child>
                  <widget class="GtkHSeparator" id="hseparator1"><xsl:attribute name="id">hseparator1_int_<xsl:value-of select="$name" /></xsl:attribute>
                    <property name="visible">True</property>
                  </widget>
                  <packing>
                    <property name="right_attach">9</property>
                    <property name="top_attach">1</property>
                    <property name="bottom_attach">2</property>
                  </packing>
                </child>
                <child>
                  <widget class="GtkVSeparator" id="vseparator1"><xsl:attribute name="id">vseparator1_int_<xsl:value-of select="$name" /></xsl:attribute>
                    <property name="visible">True</property>
                    <property name="orientation">GTK_ORIENTATION_VERTICAL</property>
                  </widget>
                  <packing>
                    <property name="left_attach">4</property>
                    <property name="right_attach">5</property>
                    <property name="bottom_attach"><xsl:value-of select="$motorsNo + 3" /></property> <!-- 3 + RN -->
                  </packing>
                </child>
                <child>
                  <widget class="GtkHSeparator" id="hseparator2"><xsl:attribute name="id">hseparator2_int_<xsl:value-of select="$name" /></xsl:attribute>
                    <property name="visible">True</property>
                  </widget>
                  <packing>
                    <property name="right_attach">9</property>
                    <property name="top_attach"><xsl:value-of select="$motorsNo + 2" /></property> <!-- 2 + RN -->
                    <property name="bottom_attach"><xsl:value-of select="$motorsNo + 3" /></property> <!-- 3 + RN -->
                  </packing>
                </child>
                <child>
                  <widget class="GtkVSeparator" id="vseparator2"><xsl:attribute name="id">vseparator2_int_<xsl:value-of select="$name" /></xsl:attribute>
                    <property name="visible">True</property>
                    <property name="orientation">GTK_ORIENTATION_VERTICAL</property>
                  </widget>
                  <packing>
                    <property name="left_attach">6</property>
                    <property name="right_attach">7</property>
                    <property name="bottom_attach"><xsl:value-of select="$motorsNo + 3" /></property> <!-- 3 + RN -->
                  </packing>
                </child>
                <child>
                  <widget class="GtkVSeparator" id="vseparator3"><xsl:attribute name="id">vseparator3_int_<xsl:value-of select="$name" /></xsl:attribute>
                    <property name="visible">True</property>
                    <property name="orientation">GTK_ORIENTATION_VERTICAL</property>
                  </widget>
                  <packing>
                    <property name="left_attach">1</property>
                    <property name="right_attach">2</property>
                    <property name="bottom_attach"><xsl:value-of select="$motorsNo + 3" /></property> <!-- 3 + RN -->
                  </packing>
                </child>
<!-- call loop for each position -->
		<xsl:call-template name="for.each.edp.irp6.int">
    		<xsl:with-param name="motorsNo" select="$motorsNo"/>
    		<xsl:with-param name="fullName" select="$fullName"/>
			<xsl:with-param name="i" select="1"/>
			<xsl:with-param name="name" select="$name"/>
 		</xsl:call-template>
<!-- end tags -->
              </widget>
            </child>
          </widget>
        </child>
      </widget>
    </child>
  </widget>
</xsl:template>


<!-- irp6 int repeatable part -->
<xsl:template name="for.each.edp.irp6.int">
<xsl:param name="motorsNo"/>
<xsl:param name="fullName"/>
<xsl:param name="i"/>
<xsl:param name="name"/>
	<xsl:if test="$i &lt;= $motorsNo">
                <child>
                  <widget class="GtkLabel" id="label5"><xsl:attribute name="id">label<xsl:value-of select="$i"/>_int_<xsl:value-of select="$name" /></xsl:attribute> <!-- RI --> 
                    <property name="visible">True</property>
                    <property name="label" translatable="yes">Θ<xsl:value-of select="$i"/></property> <!-- RI --> 
                  </widget>
                  <packing>
                    <property name="top_attach"><xsl:value-of select="$i + 1"/></property> <!-- 1 + RI -->
                    <property name="bottom_attach"><xsl:value-of select="$i + 2"/></property> <!-- 2 + RI -->
                  </packing>
                </child>
                <child>
                  <widget class="GtkEntry" id="entry4"><xsl:attribute name="id">entry<xsl:value-of select="$i"/>_int_<xsl:value-of select="$name" /></xsl:attribute> <!-- RI --> 
                    <property name="width_request">66</property>
                    <property name="visible">True</property>
                    <property name="can_focus">True</property>
                    <property name="editable">False</property>
                    <property name="text" translatable="yes">0.000</property>
                  </widget>
                  <packing>
                    <property name="left_attach">2</property>
                    <property name="right_attach">3</property>
                    <property name="top_attach"><xsl:value-of select="$i + 1"/></property>  <!-- 1 + RI -->
                    <property name="bottom_attach"><xsl:value-of select="$i + 2"/></property> <!-- 2 + RI -->
                  </packing>
                </child>
                <child>
                  <widget class="GtkSpinButton" id="spinbutton6"><xsl:attribute name="id">spinbutton<xsl:value-of select="$i"/>_int_<xsl:value-of select="$name" /></xsl:attribute> <!--RI--> 
                    <property name="width_request">50</property>
                    <property name="visible">True</property>
                    <property name="can_focus">True</property>
                    <property name="adjustment">0 -999999999999999 999999999999999 0.01 0.1 0</property>
                    <property name="digits">3</property>
                  </widget>
                  <packing>
                    <property name="left_attach">5</property>
                    <property name="right_attach">6</property>
                    <property name="top_attach"><xsl:value-of select="$i + 1"/></property>  <!-- 1 + RI -->
                    <property name="bottom_attach"><xsl:value-of select="$i + 2"/></property> <!-- 2 + RI -->
                  </packing>
                </child>
                <child>
                  <widget class="GtkHButtonBox" id="hbuttonbox4"><xsl:attribute name="id">hbuttonbox<xsl:value-of select="$i"/>_int_<xsl:value-of select="$name" /></xsl:attribute> <!--RI--> 
                    <property name="width_request">150</property>
                    <property name="visible">True</property>
                    <property name="layout_style">GTK_BUTTONBOX_CENTER</property>
                    <child>
                      <widget class="GtkButton" id="button11"><xsl:attribute name="id">button<xsl:value-of select="($i*2)-1"/>_int_<xsl:value-of select="$name" /></xsl:attribute> <!--RIx2--> 
                        <property name="visible">True</property>
                        <property name="can_focus">True</property>
                        <property name="receives_default">True</property>
                        <property name="response_id">0</property>
                        <signal name="clicked" handler="on_button{($i*2)-1}_clicked_{$fullName}_int"/>
                        <child>
                          <widget class="GtkArrow" id="arrow10"><xsl:attribute name="id">arrow<xsl:value-of select="($i*2)-1"/>_int_<xsl:value-of select="$name" /></xsl:attribute> <!-- RI x 2 --> 
                            <property name="visible">True</property>
                            <property name="arrow_type">GTK_ARROW_LEFT</property>
                          </widget>
                        </child>
                      </widget>
                    </child>
                    <child>
                      <widget class="GtkButton" id="button12"><xsl:attribute name="id">button<xsl:value-of select="($i*2)"/>_int_<xsl:value-of select="$name" /></xsl:attribute> <!-- RI x 2 -1 --> 
                        <property name="visible">True</property>
                        <property name="can_focus">True</property>
                        <property name="receives_default">True</property>
                        <property name="response_id">0</property>
                        <signal name="clicked" handler="on_button{($i*2)}_clicked_{$fullName}_int"/>
                        <child>
                          <widget class="GtkArrow" id="arrow5"><xsl:attribute name="id">arrow<xsl:value-of select="($i*2)"/>_int_<xsl:value-of select="$name" /></xsl:attribute> <!-- RI x 2 -1 --> 
                            <property name="visible">True</property>
                          </widget>
                        </child>
                      </widget>
                      <packing>
                        <property name="position">1</property>
                      </packing>
                    </child>
                  </widget>
                  <packing>
                    <property name="left_attach">7</property>
                    <property name="right_attach">9</property>
                    <property name="top_attach"><xsl:value-of select="$i + 1"/></property>  <!-- 1 + RI -->
                    <property name="bottom_attach"><xsl:value-of select="$i + 2"/></property> <!-- 2 + RI -->
                  </packing>
                </child>
                <child>
                  <widget class="GtkLabel" id="labelSpace"><xsl:attribute name="id">labelSpace_int_<xsl:value-of select="$name" />_<xsl:value-of select="$i" /></xsl:attribute>
                    <property name="width_request">36</property>
                    <property name="visible">True</property>
                  </widget>
                  <packing>
                    <property name="left_attach">6</property>
                    <property name="right_attach">7</property>
                    <property name="top_attach"><xsl:value-of select="$motorsNo + 3" /></property>
                    <property name="bottom_attach"><xsl:value-of select="$motorsNo + 4" /></property>
                  </packing>
                </child>
       </xsl:if>
	<!-- for loop --> 
       <xsl:if test="$i &lt;= $motorsNo">
          <xsl:call-template name="for.each.edp.irp6.int">
              <xsl:with-param name="i">
                  <xsl:value-of select="$i + 1"/>
              </xsl:with-param>
              <xsl:with-param name="motorsNo">
                  <xsl:value-of select="$motorsNo"/>
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
