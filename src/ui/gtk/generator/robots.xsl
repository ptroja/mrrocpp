<?xml version="1.0" encoding="UTF-8"?>
<!--
MRROC++ GUI generator
Main generator file - includes xsl files for each window
 -->
<xsl:stylesheet version="1.0" xmlns:xsl="http://www.w3.org/1999/XSL/Transform">
<xsl:output method="xml" doctype-system="glade-2.0.dtd" indent="yes" version="1.0"/>

<xsl:include href="edp_irp6.xsl" />
<xsl:include href="edp_irp6_signals.xsl" />
<xsl:include href="servo_algorithm.xsl" />
<xsl:include href="servo_algorithm_signals.xsl" />
<xsl:include href="int.xsl" />
<xsl:include href="int_signals.xsl" />
<xsl:include href="inc.xsl" />
<xsl:include href="inc_signals.xsl" />
<xsl:include href="axis_xyz.xsl" />
<xsl:include href="axis_xyz_signals.xsl" />
<xsl:include href="axis_ts.xsl" />
<xsl:include href="axis_ts_signals.xsl" />
<xsl:include href="euler_xyz.xsl" />
<xsl:include href="euler_xyz_signals.xsl" />
<xsl:include href="euler_ts.xsl" />
<xsl:include href="euler_ts_signals.xsl" />
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
<xsl:variable name="axis_xyz" select="axis_xyz"/>
	<xsl:if test="$axis_xyz &gt; 0">	
		<xsl:call-template name="irp6.axis.xyz" />
	</xsl:if>
<xsl:variable name="axis_ts" select="axis_ts"/>
	<xsl:if test="$axis_ts &gt; 0">
		<xsl:call-template name="irp6.axis.ts" />
	</xsl:if>
<xsl:variable name="euler_xyz" select="euler_xyz"/>
	<xsl:if test="$euler_xyz &gt; 0">
		<xsl:call-template name="irp6.euler.xyz" />
	</xsl:if>
<xsl:variable name="euler_ts" select="euler_ts"/>
	<xsl:if test="$euler_ts &gt; 0">
		<xsl:call-template name="irp6.euler.ts" />
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
