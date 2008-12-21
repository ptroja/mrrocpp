<?xml version="1.0" encoding="UTF-8"?>
<!--
MRROC++ GUI generator
Inc window, in the end of window creation there are call-templates attributes to templates for other windows
 -->
<xsl:stylesheet version="1.0" xmlns:xsl="http://www.w3.org/1999/XSL/Transform">
<xsl:output method="xml" doctype-system="glade-2.0.dtd" indent="yes" version="1.0"/>


<!-- main irp6 inc part -->
<xsl:template name="irp6.inc" match="*[substring(name(),1,4)='irp6']">
<xsl:variable name="name" select="name"/>
<xsl:variable name="irp6EDPNumber" select="irp6EDPNumber"/>
<xsl:document method="xml" doctype-system="glade-2.0.dtd" indent="yes" version="1.0" href="../{$name}inc.glade">
<glade-interface>
  <widget class="GtkWindow" id="window">
    <child>
      <widget class="GtkScrolledWindow" id="scrolledwindow1">
        <property name="visible">True</property>
        <property name="can_focus">True</property>
        <property name="hscrollbar_policy">GTK_POLICY_AUTOMATIC</property>
        <property name="vscrollbar_policy">GTK_POLICY_AUTOMATIC</property>
        <child>
          <widget class="GtkViewport" id="viewport1">
            <property name="visible">True</property>
            <property name="resize_mode">GTK_RESIZE_QUEUE</property>
            <child>
              <widget class="GtkTable" id="table1">
                <property name="visible">True</property>
                <property name="n_rows"><xsl:value-of select="$irp6EDPNumber + 5" /></property> <!-- 5 + RN  -->
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
                  <widget class="GtkSpinButton" id="spinbuttonDown1">
                    <property name="visible">True</property>
                    <property name="can_focus">True</property>
                    <property name="adjustment">0 0 100 1 10 10</property>
                    <property name="digits">3</property> 
                  </widget>
                  <packing>
                    <property name="left_attach">7</property>
                    <property name="right_attach">9</property>
                    <property name="top_attach"><xsl:value-of select="$irp6EDPNumber + 3" /></property>  <!-- 3 + RN  -->
                    <property name="bottom_attach"><xsl:value-of select="$irp6EDPNumber + 4" /></property>  <!-- 4 + RN  -->
                  </packing>
                </child>
                <child>
                  <widget class="GtkButton" id="buttonDown1">
                    <property name="visible">True</property>
                    <property name="can_focus">True</property>
                    <property name="receives_default">True</property>
                    <property name="label" translatable="yes">Execute move</property>
                    <property name="response_id">0</property>
                  </widget>
                  <packing>
                    <property name="left_attach">4</property>
                    <property name="right_attach">6</property>
                    <property name="top_attach"><xsl:value-of select="$irp6EDPNumber + 3" /></property>  <!-- 3 + RN  -->
                    <property name="bottom_attach"><xsl:value-of select="$irp6EDPNumber + 4" /></property> <!-- 4 + RN  -->
                  </packing>
                </child>
                <child>
                  <widget class="GtkButton" id="buttonDown2">
                    <property name="visible">True</property>
                    <property name="can_focus">True</property>
                    <property name="receives_default">True</property>
                    <property name="label" translatable="yes">Import</property>
                    <property name="response_id">0</property>
                  </widget>
                  <packing>
                    <property name="left_attach">4</property>
                    <property name="right_attach">5</property>
                    <property name="top_attach"><xsl:value-of select="$irp6EDPNumber + 4" /></property>  <!-- 4 + RN  -->
                    <property name="bottom_attach"><xsl:value-of select="$irp6EDPNumber + 5" /></property>  <!-- 5 + RN  -->
                  </packing>
                </child>
                <child>
                  <widget class="GtkButton" id="buttonLeft1">
                    <property name="visible">True</property>
                    <property name="can_focus">True</property>
                    <property name="receives_default">True</property>
                    <property name="response_id">0</property>
                    <child>
                      <widget class="GtkArrow" id="arrowLeft1">
                        <property name="visible">True</property>
                      </widget>
                    </child>
                  </widget>
                  <packing>
                    <property name="left_attach">3</property>
                    <property name="right_attach">4</property>
                    <property name="top_attach">2</property>
                    <property name="bottom_attach"><xsl:value-of select="$irp6EDPNumber + 2" /></property> <!-- 2 + RN  -->
                    <property name="x_options"></property>
                    <property name="x_padding">5</property>
                  </packing>
                </child>
                <child>
                  <widget class="GtkButton" id="buttonDown3">
                    <property name="visible">True</property>
                    <property name="can_focus">True</property>
                    <property name="receives_default">True</property>
                    <property name="label" translatable="yes">Read</property>
                    <property name="response_id">0</property>
                  </widget>
                  <packing>
                    <property name="left_attach">2</property>
                    <property name="right_attach">3</property>
                    <property name="top_attach"><xsl:value-of select="$irp6EDPNumber + 3" /></property>  <!-- 3 + RN  -->
                    <property name="bottom_attach"><xsl:value-of select="$irp6EDPNumber + 4" /></property>  <!-- 4 + RN  -->
                  </packing>
                </child>
                <child>
                  <widget class="GtkButton" id="buttonDown4">
                    <property name="visible">True</property>
                    <property name="can_focus">True</property>
                    <property name="receives_default">True</property>
                    <property name="label" translatable="yes">Export</property>
                    <property name="response_id">0</property>
                  </widget>
                  <packing>
                    <property name="left_attach">5</property>
                    <property name="right_attach">6</property>
                    <property name="top_attach"><xsl:value-of select="$irp6EDPNumber + 4" /></property> <!-- 4 + RN  -->
                    <property name="bottom_attach"><xsl:value-of select="$irp6EDPNumber + 5" /></property> <!-- 5 + RN  -->
                  </packing>
                </child>
                <child>
                  <widget class="GtkLabel" id="labelDown1">
                    <property name="visible">True</property>
                    <property name="label" translatable="yes">step</property>
                  </widget>
                  <packing>
                    <property name="left_attach">7</property>
                    <property name="right_attach">9</property>
                    <property name="top_attach"><xsl:value-of select="$irp6EDPNumber + 4" /></property>  <!-- 4 + RN  -->
                    <property name="bottom_attach"><xsl:value-of select="$irp6EDPNumber + 5" /></property>  <!-- 5 + RN  -->
                  </packing>
                </child>
                <child>
                  <widget class="GtkLabel" id="labelUp1">
                    <property name="visible">True</property>
                    <property name="label" translatable="yes">axis</property>
                  </widget>
                </child>
                <child>
                  <widget class="GtkLabel" id="labelUp2">
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
                  <widget class="GtkLabel" id="labelUp3">
                    <property name="visible">True</property>
                    <property name="label" translatable="yes">incremental move</property>
                  </widget>
                  <packing>
                    <property name="left_attach">7</property>
                    <property name="right_attach">9</property>
                  </packing>
                </child>
                <child>
                  <widget class="GtkLabel" id="labelUp4">
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
                  <widget class="GtkHSeparator" id="hseparator1">
                    <property name="visible">True</property>
                  </widget>
                  <packing>
                    <property name="right_attach">9</property>
                    <property name="top_attach">1</property>
                    <property name="bottom_attach">2</property>
                  </packing>
                </child>
                <child>
                  <widget class="GtkVSeparator" id="vseparator1">
                    <property name="visible">True</property>
                  </widget>
                  <packing>
                    <property name="left_attach">4</property>
                    <property name="right_attach">5</property>
                    <property name="bottom_attach"><xsl:value-of select="$irp6EDPNumber + 3" /></property> <!-- 3 + RN -->
                  </packing>
                </child>
                <child>
                  <widget class="GtkHSeparator" id="hseparator2">
                    <property name="visible">True</property>
                  </widget>
                  <packing>
                    <property name="right_attach">9</property>
                    <property name="top_attach"><xsl:value-of select="$irp6EDPNumber + 2" /></property> <!-- 2 + RN -->
                    <property name="bottom_attach"><xsl:value-of select="$irp6EDPNumber + 3" /></property> <!-- 3 + RN -->
                  </packing>
                </child>
                <child>
                  <widget class="GtkVSeparator" id="vseparator2">
                    <property name="visible">True</property>
                  </widget>
                  <packing>
                    <property name="left_attach">6</property>
                    <property name="right_attach">7</property>
                    <property name="bottom_attach"><xsl:value-of select="$irp6EDPNumber + 3" /></property> <!-- 3 + RN -->
                  </packing>
                </child>
                <child>
                  <widget class="GtkVSeparator" id="vseparator3">
                    <property name="visible">True</property>
                  </widget>
                  <packing>
                    <property name="left_attach">1</property>
                    <property name="right_attach">2</property>
                    <property name="bottom_attach"><xsl:value-of select="$irp6EDPNumber + 3" /></property> <!-- 3 + RN -->
                  </packing>
                </child>
<!-- call loop for each position -->
		<xsl:call-template name="for.each.edp.irp6.inc">
    			<xsl:with-param name="irp6EDPNumber" select="$irp6EDPNumber"/>
			<xsl:with-param name="i" select="1"/>
 		</xsl:call-template>
<!-- end tags -->
              </widget>
            </child>
          </widget>
        </child>
      </widget>
    </child>
  </widget>
</glade-interface>
</xsl:document>
<!-- call other templates windows -->
<xsl:call-template name="irp6.servo" />
<xsl:call-template name="irp6.int" />
<xsl:call-template name="irp6.axis.xyz" />
<xsl:variable name="axis_ts" select="axis_ts"/>
<xsl:if test="$axis_ts &gt;= 0"><xsl:call-template name="irp6.axis.ts" /></xsl:if>
<xsl:call-template name="irp6.euler.xyz" />
<xsl:variable name="euler_ts" select="euler_ts"/>
<xsl:if test="$euler_ts &gt;= 0"><xsl:call-template name="irp6.euler.ts" /></xsl:if>
<xsl:call-template name="conveyor.servo" />
<xsl:call-template name="conveyor.moves" />
</xsl:template>


<!-- irp6 inc repeatable part -->
<xsl:template name="for.each.edp.irp6.inc">
<xsl:param name="irp6EDPNumber"/>
<xsl:param name="i"/>
	<xsl:if test="$i &lt;= $irp6EDPNumber">
                <child>
                  <widget class="GtkLabel" id="label5"><xsl:attribute name="id">label<xsl:value-of select="$i"/></xsl:attribute> <!-- RI --> 
                    <property name="visible">True</property>
                    <property name="label" translatable="yes"><xsl:value-of select="$i - 1"/></property> <!-- RI --> 
                  </widget>
                  <packing>
                    <property name="top_attach"><xsl:value-of select="$i + 1"/></property> <!-- 1 + RI -->
                    <property name="bottom_attach"><xsl:value-of select="$i + 2"/></property> <!-- 2 + RI -->
                  </packing>
                </child>
                <child>
                  <widget class="GtkEntry" id="entry4"><xsl:attribute name="id">entry<xsl:value-of select="$i"/></xsl:attribute> <!-- RI --> 
                    <property name="width_request">66</property>
                    <property name="visible">True</property>
                    <property name="can_focus">True</property>
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
                  <widget class="GtkSpinButton" id="spinbutton6"><xsl:attribute name="id">spinbutton<xsl:value-of select="$i"/></xsl:attribute> <!--RI--> 
                    <property name="visible">True</property>
                    <property name="can_focus">True</property>
                    <property name="adjustment">0 0 100 1 10 10</property>
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
                  <widget class="GtkHButtonBox" id="hbuttonbox4"><xsl:attribute name="id">hbuttonbox<xsl:value-of select="$i"/></xsl:attribute> <!--RI--> 
                    <property name="width_request">150</property>
                    <property name="visible">True</property>
                    <property name="layout_style">GTK_BUTTONBOX_CENTER</property>
                    <child>
                      <widget class="GtkButton" id="button11"><xsl:attribute name="id">button<xsl:value-of select="($i*2)-1"/></xsl:attribute> <!--RIx2--> 
                        <property name="visible">True</property>
                        <property name="can_focus">True</property>
                        <property name="receives_default">True</property>
                        <property name="response_id">0</property>
                        <child>
                          <widget class="GtkArrow" id="arrow10"><xsl:attribute name="id">arrow<xsl:value-of select="($i*2)-1"/></xsl:attribute> <!-- RI x 2 --> 
                            <property name="visible">True</property>
                            <property name="arrow_type">GTK_ARROW_LEFT</property>
                          </widget>
                        </child>
                      </widget>
                    </child>
                    <child>
                      <widget class="GtkButton" id="button12"><xsl:attribute name="id">button<xsl:value-of select="($i*2)"/></xsl:attribute> <!-- RI x 2 -1 --> 
                        <property name="visible">True</property>
                        <property name="can_focus">True</property>
                        <property name="receives_default">True</property>
                        <property name="response_id">0</property>
                        <child>
                          <widget class="GtkArrow" id="arrow5"><xsl:attribute name="id">arrow<xsl:value-of select="($i*2)"/></xsl:attribute> <!-- RI x 2 -1 --> 
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
       <xsl:if test="$i &lt;= $irp6EDPNumber">
          <xsl:call-template name="for.each.edp.irp6.inc">
              <xsl:with-param name="i">
                  <xsl:value-of select="$i + 1"/>
              </xsl:with-param>
              <xsl:with-param name="irp6EDPNumber">
                  <xsl:value-of select="$irp6EDPNumber"/>
              </xsl:with-param>
          </xsl:call-template>
       </xsl:if>
</xsl:template>

</xsl:stylesheet>

