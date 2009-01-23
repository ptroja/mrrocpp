<?xml version="1.0" encoding="UTF-8"?>
<!--
MRROC++ GUI generator
Timestamp file
 -->
<xsl:stylesheet version="1.0" xmlns:xsl="http://www.w3.org/1999/XSL/Transform">
<xsl:output method="xml" doctype-system="glade-2.0.dtd" indent="yes" version="1.0"/>


<!-- timestamp file -->
<xsl:template name="timestamp.file" match="festival">
<xsl:document method="xml" doctype-system="glade-2.0.dtd" indent="yes" version="1.0" href="../timestamp.xml">

<timestamp>
<!-- 
 <xsl:value-of select="current-dateTime()"/>
--> 
 <xsl:value-of select="system-property('xsl:vendor')"/>
</timestamp>

</xsl:document>
</xsl:template>
</xsl:stylesheet>
