<?xml version="1.0" encoding="UTF-8"?>
<!--
MRROC++ GUI generator
Main generator file - includes xsl files for each window
 -->
<xsl:stylesheet version="1.0" xmlns:xsl="http://www.w3.org/1999/XSL/Transform">
<xsl:output method="xml" doctype-system="glade-2.0.dtd" indent="yes" version="1.0"/>

<xsl:include href="servo_algorithm.xsl" />
<xsl:include href="int.xsl" />
<xsl:include href="axis_xyz.xsl" />
<xsl:include href="axis_ts.xsl" />
<xsl:include href="euler_xyz.xsl" />
<xsl:include href="euler_ts.xsl" />
<xsl:include href="conveyor.xsl" />
<xsl:include href="inc.xsl" />

</xsl:stylesheet>
