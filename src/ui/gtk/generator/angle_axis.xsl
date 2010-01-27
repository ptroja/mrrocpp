<?xml version="1.0" encoding="UTF-8"?>
<!--
MRROC++ GUI generator
xyz_angle_axis window
 -->
<xsl:stylesheet version="1.0" xmlns:xsl="http://www.w3.org/1999/XSL/Transform">
<xsl:output method="xml" doctype-system="glade-2.0.dtd" indent="yes" version="1.0"/>

<!-- main irp6 xyz_angle_axis part -->
<xsl:template name="irp6.xyz_angle_axis" match="*[substring(name(),1,4)='irp6']">
<xsl:variable name="name" select="name"/>
<xsl:variable name="fullName" select="fullName"/>
<xsl:variable name="xyz_angle_axis" select="xyz_angle_axis"/>
  <widget class="GtkWindow" id="window_xyz_angle_axis">
    <child>
      <widget class="GtkScrolledWindow" id="scrolledwindow1"><xsl:attribute name="id">scrolledwindow_xyz_angle_axis_<xsl:value-of select="$name" /></xsl:attribute>
        <property name="visible">True</property>
        <property name="can_focus">True</property>
        <property name="hscrollbar_policy">GTK_POLICY_AUTOMATIC</property>
        <property name="vscrollbar_policy">GTK_POLICY_AUTOMATIC</property>
        <child>
          <widget class="GtkViewport" id="viewport1"><xsl:attribute name="id">viewport_xyz_angle_axis_<xsl:value-of select="$name" /></xsl:attribute>
            <property name="visible">True</property>
            <property name="resize_mode">GTK_RESIZE_QUEUE</property>
            <child>
              <widget class="GtkTable" id="table1"><xsl:attribute name="id">table_xyz_angle_axis_<xsl:value-of select="$name" /></xsl:attribute>
                <property name="visible">True</property>
                <property name="n_rows"><xsl:value-of select="$xyz_angle_axis + 5" /></property> <!-- 5 + RN  -->
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
                  <widget class="GtkSpinButton" id="spinbuttonDown1"><xsl:attribute name="id">spinbuttonDown1_xyz_angle_axis_<xsl:value-of select="$name" /></xsl:attribute>
                  	<property name="width_request">50</property>
                    <property name="visible">True</property>
                    <property name="can_focus">True</property>
                    <property name="adjustment">0 -999999999999999 999999999999999 1 10 0</property>
                    <property name="digits">3</property>
                    <property name="numeric">True</property>
                    <property name="overwrite_mode">True</property>
                  </widget>
                  <packing>
                    <property name="left_attach">7</property>
                    <property name="right_attach">9</property>
                    <property name="top_attach"><xsl:value-of select="$xyz_angle_axis + 3" /></property>  <!-- 3 + RN  -->
                    <property name="bottom_attach"><xsl:value-of select="$xyz_angle_axis + 4" /></property>  <!-- 4 + RN  -->
                  </packing>
                </child>
                <child>
                  <widget class="GtkButton" id="buttonDown1"><xsl:attribute name="id">buttonDown1_xyz_angle_axis_<xsl:value-of select="$name" /></xsl:attribute>
                    <property name="visible">True</property>
                    <property name="can_focus">True</property>
                    <property name="receives_default">True</property>
                    <property name="label" translatable="yes">Execute motion</property>
                    <property name="response_id">0</property>
                    <signal name="clicked" handler="on_execute_button_clicked_{$fullName}_xyz_angle_axis"/>
                  </widget>
                  <packing>
                    <property name="left_attach">5</property>
                    <property name="right_attach">6</property>
                    <property name="top_attach"><xsl:value-of select="$xyz_angle_axis + 3" /></property>  <!-- 3 + RN  -->
                    <property name="bottom_attach"><xsl:value-of select="$xyz_angle_axis + 4" /></property> <!-- 4 + RN  -->
                  </packing>
                </child>
                <child>
                  <widget class="GtkButton" id="buttonLeft1"><xsl:attribute name="id">buttonLeft1_xyz_angle_axis_<xsl:value-of select="$name" /></xsl:attribute>
                    <property name="visible">True</property>
                    <property name="can_focus">True</property>
                    <property name="receives_default">True</property>
                    <property name="response_id">0</property>
                    <signal name="clicked" handler="on_arrow_button_clicked_{$fullName}_xyz_angle_axis"/>
                    <child>
                      <widget class="GtkArrow" id="arrowLeft1"><xsl:attribute name="id">arrowLeft1_xyz_angle_axis_<xsl:value-of select="$name" /></xsl:attribute>
                        <property name="visible">True</property>
                      </widget>
                    </child>
                  </widget>
                  <packing>
                    <property name="left_attach">3</property>
                    <property name="right_attach">4</property>
                    <property name="top_attach">2</property>
                    <property name="bottom_attach"><xsl:value-of select="$xyz_angle_axis + 2" /></property> <!-- 2 + RN  -->
                    <property name="x_options"></property>
                    <property name="x_padding">5</property>
                  </packing>
                </child>
                <child>
                  <widget class="GtkButton" id="buttonDown3"><xsl:attribute name="id">buttonDown3_xyz_angle_axis_<xsl:value-of select="$name" /></xsl:attribute>
                    <property name="visible">True</property>
                    <property name="can_focus">True</property>
                    <property name="receives_default">True</property>
                    <property name="label" translatable="yes">Read</property>
                    <property name="response_id">0</property>
                    <signal name="clicked" handler="on_read_button_clicked_{$fullName}_xyz_angle_axis"/>
                  </widget>
                  <packing>
                    <property name="left_attach">2</property>
                    <property name="right_attach">3</property>
                    <property name="top_attach"><xsl:value-of select="$xyz_angle_axis + 3" /></property>  <!-- 3 + RN  -->
                    <property name="bottom_attach"><xsl:value-of select="$xyz_angle_axis + 4" /></property>  <!-- 4 + RN  -->
                  </packing>
                </child>
                <child>
                  <widget class="GtkLabel" id="labelDown1"><xsl:attribute name="id">labelDown1_xyz_angle_axis_<xsl:value-of select="$name" /></xsl:attribute>
                    <property name="visible">True</property>
                    <property name="label" translatable="yes">step</property>
                  </widget>
                  <packing>
                    <property name="left_attach">7</property>
                    <property name="right_attach">9</property>
                    <property name="top_attach"><xsl:value-of select="$xyz_angle_axis + 4" /></property>  <!-- 4 + RN  -->
                    <property name="bottom_attach"><xsl:value-of select="$xyz_angle_axis + 5" /></property>  <!-- 5 + RN  -->
                  </packing>
                </child>
                <child>
                  <widget class="GtkLabel" id="labelUp1"><xsl:attribute name="id">labelUp1_xyz_angle_axis_<xsl:value-of select="$name" /></xsl:attribute>
                    <property name="visible">True</property>
                    <property name="label" translatable="yes">axis</property>
                  </widget>
                </child>
                <child>
                  <widget class="GtkLabel" id="labelUp2"><xsl:attribute name="id">labelUp2_xyz_angle_axis_<xsl:value-of select="$name" /></xsl:attribute>
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
                  <widget class="GtkLabel" id="labelUp3"><xsl:attribute name="id">labelUp3_xyz_angle_axis_<xsl:value-of select="$name" /></xsl:attribute>
                    <property name="visible">True</property>
                    <property name="label" translatable="yes">incremental move</property>
                  </widget>
                  <packing>
                    <property name="left_attach">7</property>
                    <property name="right_attach">9</property>
                  </packing>
                </child>
                <child>
                  <widget class="GtkLabel" id="labelUp4"><xsl:attribute name="id">labelUp4_xyz_angle_axis_<xsl:value-of select="$name" /></xsl:attribute>
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
                  <widget class="GtkHSeparator" id="hseparator1"><xsl:attribute name="id">hseperator1_xyz_angle_axis_<xsl:value-of select="$name" /></xsl:attribute>
                    <property name="visible">True</property>
                  </widget>
                  <packing>
                    <property name="right_attach">9</property>
                    <property name="top_attach">1</property>
                    <property name="bottom_attach">2</property>
                  </packing>
                </child>
                <child>
                  <widget class="GtkVSeparator" id="vseparator1"><xsl:attribute name="id">vseperator1_xyz_angle_axis_<xsl:value-of select="$name" /></xsl:attribute>
                    <property name="visible">True</property>
                    <property name="orientation">GTK_ORIENTATION_VERTICAL</property>
                  </widget>
                  <packing>
                    <property name="left_attach">4</property>
                    <property name="right_attach">5</property>
                    <property name="bottom_attach"><xsl:value-of select="$xyz_angle_axis + 3" /></property> <!-- 3 + RN -->
                  </packing>
                </child>
                <child>
                  <widget class="GtkHSeparator" id="hseparator2"><xsl:attribute name="id">hseperator2_xyz_angle_axis_<xsl:value-of select="$name" /></xsl:attribute>
                    <property name="visible">True</property>
                  </widget>
                  <packing>
                    <property name="right_attach">9</property>
                    <property name="top_attach"><xsl:value-of select="$xyz_angle_axis + 2" /></property> <!-- 2 + RN -->
                    <property name="bottom_attach"><xsl:value-of select="$xyz_angle_axis + 3" /></property> <!-- 3 + RN -->
                  </packing>
                </child>
                <child>
                  <widget class="GtkVSeparator" id="vseparator2"><xsl:attribute name="id">vseperator2_xyz_angle_axis_<xsl:value-of select="$name" /></xsl:attribute>
                    <property name="visible">True</property>
                    <property name="orientation">GTK_ORIENTATION_VERTICAL</property>
                  </widget>
                  <packing>
                    <property name="left_attach">6</property>
                    <property name="right_attach">7</property>
                    <property name="bottom_attach"><xsl:value-of select="$xyz_angle_axis + 3" /></property> <!-- 3 + RN -->
                  </packing>
                </child>
                <child>
                  <widget class="GtkVSeparator" id="vseparator3"><xsl:attribute name="id">vseperator3_xyz_angle_axis_<xsl:value-of select="$name" /></xsl:attribute>
                    <property name="visible">True</property>
                    <property name="orientation">GTK_ORIENTATION_VERTICAL</property>
                  </widget>
                  <packing>
                    <property name="left_attach">1</property>
                    <property name="right_attach">2</property>
                    <property name="bottom_attach"><xsl:value-of select="$xyz_angle_axis + 3" /></property> <!-- 3 + RN -->
                  </packing>
                </child>
                <child>
                  <widget class="GtkLabel" id="labelSpace"><xsl:attribute name="id">labelSpace_xyz_angle_axis_<xsl:value-of select="$name" /></xsl:attribute>
                    <property name="width_request">36</property>
                    <property name="visible">True</property>
                  </widget>
                  <packing>
                    <property name="left_attach">6</property>
                    <property name="right_attach">7</property>
                    <property name="top_attach"><xsl:value-of select="$xyz_angle_axis + 3" /></property>
                    <property name="bottom_attach"><xsl:value-of select="$xyz_angle_axis + 4" /></property>
                  </packing>
                </child>
<!-- call loop for each position -->
		<xsl:call-template name="for.each.edp.irp6.xyz_angle_axis">
    		<xsl:with-param name="xyz_angle_axis" select="$xyz_angle_axis"/>
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


<!-- irp6 xyz_angle_axis repeatable part -->
<xsl:template name="for.each.edp.irp6.xyz_angle_axis">
<xsl:param name="xyz_angle_axis"/>
<xsl:param name="fullName"/>
<xsl:param name="name"/>
<xsl:param name="i"/>
	<xsl:if test="$i &lt;= $xyz_angle_axis">
                <child>
                  <widget class="GtkLabel" id="label5"><xsl:attribute name="id">label<xsl:value-of select="$i"/>_xyz_angle_axis_<xsl:value-of select="$name"/></xsl:attribute> <!-- RI --> 
                    <property name="visible">True</property>
                    <property name="label" translatable="yes">
						<xsl:if test="$i = 1">X</xsl:if>
						<xsl:if test="$i = 2">Y</xsl:if>
						<xsl:if test="$i = 3">Z</xsl:if>
						<xsl:if test="$i = 4">w&lt;sub&gt;1&lt;/sub&gt;</xsl:if>
						<xsl:if test="$i = 5">w&lt;sub&gt;2&lt;/sub&gt;</xsl:if>
						<xsl:if test="$i = 6">w&lt;sub&gt;3&lt;/sub&gt;</xsl:if>
						<xsl:if test="$i = 7">α</xsl:if>
						<xsl:if test="$i = 8">G</xsl:if>
						<xsl:if test="$i = 9">T</xsl:if>
						<xsl:if test="$i &gt; 9"> Please name me in xsl file </xsl:if>
             	    </property> <!-- RI --> 
 	    			<xsl:if test="$i = 4"><property name="use_markup">True</property></xsl:if>
 	    			<xsl:if test="$i = 5"><property name="use_markup">True</property></xsl:if>
 	    			<xsl:if test="$i = 6"><property name="use_markup">True</property></xsl:if>
                  </widget>
                  <packing>
                    <property name="top_attach"><xsl:value-of select="$i + 1"/></property> <!-- 1 + RI -->
                    <property name="bottom_attach"><xsl:value-of select="$i + 2"/></property> <!-- 2 + RI -->
                  </packing>
                </child>
                <child>
                  <widget class="GtkEntry" id="entry4"><xsl:attribute name="id">entry<xsl:value-of select="$i"/>_xyz_angle_axis_<xsl:value-of select="$name"/></xsl:attribute> <!-- RI --> 
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
                  <widget class="GtkSpinButton" id="spinbutton6"><xsl:attribute name="id">spinbutton<xsl:value-of select="$i"/>_xyz_angle_axis_<xsl:value-of select="$name"/></xsl:attribute> <!--RI--> 
                    <property name="width_request">50</property>
                    <property name="visible">True</property>
                    <property name="can_focus">True</property>
                    <property name="adjustment">0 -999999999999999 999999999999999 1 10 0</property>
                    <property name="digits">3</property>
                    <property name="numeric">True</property>
                    <property name="overwrite_mode">True</property>
                  </widget>
                  <packing>
                    <property name="left_attach">5</property>
                    <property name="right_attach">6</property>
                    <property name="top_attach"><xsl:value-of select="$i + 1"/></property>  <!-- 1 + RI -->
                    <property name="bottom_attach"><xsl:value-of select="$i + 2"/></property> <!-- 2 + RI -->
                  </packing>
                </child>
                <child>
                  <widget class="GtkHButtonBox" id="hbuttonbox4"><xsl:attribute name="id">hbuttonbox<xsl:value-of select="$i"/>_xyz_angle_axis_<xsl:value-of select="$name"/></xsl:attribute> <!--RI--> 
                    <property name="width_request">150</property>
                    <property name="visible">True</property>
                    <property name="layout_style">GTK_BUTTONBOX_CENTER</property>
                    <child>
                      <widget class="GtkButton" id="button11"><xsl:attribute name="id">button<xsl:value-of select="($i*2)-1"/>_xyz_angle_axis_<xsl:value-of select="$name"/></xsl:attribute> <!--RIx2--> 
                        <property name="visible">True</property>
                        <property name="can_focus">True</property>
                        <property name="receives_default">True</property>
                        <property name="response_id">0</property>
                        <signal name="clicked" handler="on_button{($i*2)-1}_clicked_{$fullName}_xyz_angle_axis"/>
                        <child>
                          <widget class="GtkArrow" id="arrow10"><xsl:attribute name="id">arrow<xsl:value-of select="($i*2)-1"/>_xyz_angle_axis_<xsl:value-of select="$name"/></xsl:attribute> <!-- RI x 2 --> 
                            <property name="visible">True</property>
                            <property name="arrow_type">GTK_ARROW_LEFT</property>
                          </widget>
                        </child>
                      </widget>
                    </child>
                    <child>
                      <widget class="GtkButton" id="button12"><xsl:attribute name="id">button<xsl:value-of select="($i*2)"/>_xyz_angle_axis_<xsl:value-of select="$name"/></xsl:attribute> <!-- RI x 2 -1 --> 
                        <property name="visible">True</property>
                        <property name="can_focus">True</property>
                        <property name="receives_default">True</property>
                        <property name="response_id">0</property>
                        <signal name="clicked" handler="on_button{($i*2)}_clicked_{$fullName}_xyz_angle_axis" />
                        <child>
                          <widget class="GtkArrow" id="arrow5"><xsl:attribute name="id">arrow<xsl:value-of select="($i*2)"/>_xyz_angle_axis_<xsl:value-of select="$name"/></xsl:attribute> <!-- RI x 2 -1 --> 
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
       </xsl:if>
	<!-- for loop --> 
       <xsl:if test="$i &lt;= $xyz_angle_axis">
          <xsl:call-template name="for.each.edp.irp6.xyz_angle_axis">
              <xsl:with-param name="i">
                  <xsl:value-of select="$i + 1"/>
              </xsl:with-param>
              <xsl:with-param name="xyz_angle_axis">
                  <xsl:value-of select="$xyz_angle_axis"/>
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
