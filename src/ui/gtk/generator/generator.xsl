<?xml version="1.0" encoding="UTF-8"?>
<!--
MRROC++ GUI generator
Main generator file - includes xsl files for each window
 -->
<xsl:stylesheet version="1.0" xmlns:xsl="http://www.w3.org/1999/XSL/Transform">
<xsl:output method="xml" doctype-system="glade-2.0.dtd" indent="yes" version="1.0"/>

<xsl:include href="edp_robot.xsl" />
<xsl:include href="edp_robot_signals.xsl" />
<xsl:include href="servo_algorithm.xsl" />
<xsl:include href="servo_algorithm_signals.xsl" />
<xsl:include href="int.xsl" />
<xsl:include href="int_signals.xsl" />
<xsl:include href="inc.xsl" />
<xsl:include href="inc_signals.xsl" />
<xsl:include href="angle_axis.xsl" />
<xsl:include href="angle_axis_signals.xsl" />
<xsl:include href="angle_axis_tool.xsl" />
<xsl:include href="angle_axis_tool_signals.xsl" />
<xsl:include href="euler_zyz.xsl" />
<xsl:include href="euler_zyz_signals.xsl" />
<xsl:include href="euler_zyz_tool.xsl" />
<xsl:include href="euler_zyz_tool_signals.xsl" />
<xsl:include href="kinematic.xsl" />
<xsl:include href="kinematic_signals.xsl" />
<xsl:include href="festival_signals.xsl" />
<xsl:include href="timestamp.xsl" />
<xsl:include href="festival.xsl" />

<xsl:template name="call.all.irp6.templates" match="*[substring(name(),1,4)='irp6']">
<xsl:variable name="motorsNo" select="motorsNo"/>
	<xsl:if test="$motorsNo &gt; 0">
		<xsl:call-template name="irp6.servo" />
		<xsl:call-template name="irp6.int" />
		<xsl:call-template name="irp6.inc" />
	</xsl:if>
<xsl:variable name="xyz_angle_axis" select="xyz_angle_axis"/>
	<xsl:if test="$xyz_angle_axis &gt; 0">	
		<xsl:call-template name="irp6.xyz_angle_axis" />
	</xsl:if>
<xsl:variable name="xyz_angle_axis_tool" select="xyz_angle_axis_tool"/>
	<xsl:if test="$xyz_angle_axis_tool &gt; 0">
		<xsl:call-template name="irp6.xyz_angle_axis_tool" />
	</xsl:if>
<xsl:variable name="xyz_euler_zyz" select="xyz_euler_zyz"/>
	<xsl:if test="$xyz_euler_zyz &gt; 0">
		<xsl:call-template name="irp6.xyz_euler_zyz" />
	</xsl:if>
<xsl:variable name="xyz_euler_zyz_tool" select="xyz_euler_zyz_tool"/>
	<xsl:if test="$xyz_euler_zyz_tool &gt; 0">
		<xsl:call-template name="irp6.xyz_euler_zyz_tool" />
	</xsl:if>
<xsl:variable name="kinematic" select="kinematic"/>
	<xsl:if test="$kinematic &gt; 0">
		<xsl:call-template name="irp6.kinematic" />
	</xsl:if>
<xsl:call-template name="irp6.edp.main" />
</xsl:template>

<xsl:template name="call.all.other.templates">
<xsl:call-template name="festival.main" />
<xsl:call-template name="timestamp.file" />
</xsl:template>

<!-- call templates defined in the stylesheets above -->
<xsl:template name="call.all.templates">
<xsl:call-template name="call.all.irp6.templates" />
<xsl:call-template name="call.all.other.templates" />
</xsl:template>

</xsl:stylesheet>
