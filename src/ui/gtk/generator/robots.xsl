<?xml version="1.0" encoding="UTF-8"?>
<!--
MRROC++ GUI generator
Main generator file - includes xsl files for each window
 -->
<xsl:stylesheet version="1.0" xmlns:xsl="http://www.w3.org/1999/XSL/Transform">
<xsl:output method="xml" doctype-system="glade-2.0.dtd" indent="yes" version="1.0"/>

<xsl:include href="edp_irp6_rcsc.xsl" />
<xsl:include href="edp_irp6_rcsc_signals.xsl" />
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
<xsl:include href="conveyor.xsl" />

<!-- call templates defined in the stylesheets above -->
<xsl:template name="call.all.templates" match="*[substring(name(),1,4)='irp6']">
<xsl:call-template name="irp6.servo" />
<xsl:call-template name="irp6.int" />
<xsl:call-template name="irp6.inc" />
<xsl:call-template name="irp6.axis.xyz" />
<xsl:variable name="axis_ts" select="axis_ts"/>
<xsl:if test="$axis_ts &gt;= 0"><xsl:call-template name="irp6.axis.ts" /></xsl:if>
<xsl:call-template name="irp6.euler.xyz" />
<xsl:variable name="euler_ts" select="euler_ts"/>
<xsl:if test="$euler_ts &gt;= 0"><xsl:call-template name="irp6.euler.ts" /></xsl:if>
<xsl:call-template name="conveyor.servo" />
<xsl:call-template name="conveyor.moves" />
<xsl:call-template name="irp6.edp.main" />
</xsl:template>
</xsl:stylesheet>
