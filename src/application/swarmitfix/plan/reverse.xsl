<?xml version="1.0" encoding="UTF-8"?>
<xsl:stylesheet xmlns:xsl="http://www.w3.org/1999/XSL/Transform" version="1.0">
    <xsl:template match="@*|node()">
        <xsl:copy>
            <xsl:apply-templates select="@*|node()"/>
        </xsl:copy>
    </xsl:template>
    
    <xsl:template match="item">
        <xsl:copy>            
            <xsl:copy-of select="agent"/>
            <xsl:copy-of select="TBeg"/>
            <xsl:copy-of select="TEnd"/>
            <xsl:copy-of select="@*|node()[not(name()='agent')][not(name()='TEnd')][not(name()='TBeg')]"/>            
        </xsl:copy>
    </xsl:template>

</xsl:stylesheet>