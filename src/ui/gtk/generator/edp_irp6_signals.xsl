<?xml version="1.0" encoding="UTF-8"?>
<!--
MRROC++ GUI generator
EDP IRp6 RCSC window callback signals
 -->
<xsl:stylesheet version="1.0" xmlns:xsl="http://www.w3.org/1999/XSL/Transform">
<xsl:output method="text"/>

<!-- signals handling file .cc-->
<xsl:template name="irp6.edp.main.signals.cc" match="*[substring(name(),1,4)='irp6']">
<xsl:variable name="name" select="name"/>
<xsl:variable name="fullName" select="fullName"/>
<xsl:variable name="motorsNo" select="motorsNo"/>
<xsl:variable name="axis_xyz" select="axis_xyz"/>
<xsl:variable name="axis_ts" select="axis_ts"/>
<xsl:variable name="euler_xyz" select="euler_xyz"/>
<xsl:variable name="euler_ts" select="euler_ts"/>
<xsl:document method="text" href="../signals/edp_{$name}_uimodule.cc">



<xsl:text>
#include &lt;iostream&gt;

#include &lt;gtk/gtk.h&gt;
#include &lt;glib.h&gt;

#include "ui_model.h"
#include "edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>_uimodule.h"


BYTE servo_alg_no[</xsl:text><xsl:value-of select="$motorsNo" /><xsl:text>];
BYTE servo_par_no[</xsl:text><xsl:value-of select="$motorsNo" /><xsl:text>];
	
gint servo_alg_no_tmp [</xsl:text><xsl:value-of select="$motorsNo" /><xsl:text>];
BYTE servo_alg_no_output[</xsl:text><xsl:value-of select="$motorsNo" /><xsl:text>];
gint servo_par_no_tmp [</xsl:text><xsl:value-of select="$motorsNo" /><xsl:text>];
BYTE servo_par_no_output[</xsl:text><xsl:value-of select="$motorsNo" /><xsl:text>];

char buf[32];
gchar buffer[500];
double tool_vector_a[</xsl:text><xsl:value-of select="$axis_ts" /><xsl:text>];
double tool_vector_e[</xsl:text><xsl:value-of select="$euler_ts" /><xsl:text>];
double alfa, kx, ky, kz;
double wl; 
double l_eps = 0;
double </xsl:text><xsl:value-of select="$name" /><xsl:text>_current_pos_a[</xsl:text><xsl:value-of select="$axis_xyz" /><xsl:text>]; // pozycja biezaca
double </xsl:text><xsl:value-of select="$name" /><xsl:text>_desired_pos_a[</xsl:text><xsl:value-of select="$axis_xyz" /><xsl:text>]; // pozycja zadana
double </xsl:text><xsl:value-of select="$name" /><xsl:text>_current_pos_e[</xsl:text><xsl:value-of select="$euler_xyz" /><xsl:text>]; // pozycja biezaca
double </xsl:text><xsl:value-of select="$name" /><xsl:text>_desired_pos_e[</xsl:text><xsl:value-of select="$euler_xyz" /><xsl:text>]; // pozycja zadana
double </xsl:text><xsl:value-of select="$name" /><xsl:text>_current_pos[</xsl:text><xsl:value-of select="$motorsNo" /><xsl:text>]; // pozycja biezaca
double </xsl:text><xsl:value-of select="$name" /><xsl:text>_desired_pos[</xsl:text><xsl:value-of select="$motorsNo" /><xsl:text>]; // pozycja zadana



</xsl:text><xsl:choose><xsl:when test="$name = 'conveyor'"><xsl:text>#include "ui/ui_ecp_r_conveyor.h"</xsl:text></xsl:when><xsl:otherwise><xsl:text>#include "ui/ui_ecp_r_irp6_common.h"</xsl:text></xsl:otherwise></xsl:choose><xsl:text>

edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>::edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>(ui_config_entry &amp;entry)
{
				robot = new </xsl:text><xsl:choose><xsl:when test="$name = 'conveyor'"><xsl:text>ui_conveyor_robot</xsl:text></xsl:when><xsl:otherwise><xsl:text>ui_common_robot</xsl:text></xsl:otherwise></xsl:choose><xsl:text>(
				ui_model::instance().getConfigurator(),
				&amp;ui_model::instance().getEcpSr()
				</xsl:text><xsl:choose><xsl:when test="$name = 'irp6m'"><xsl:text>,ROBOT_IRP6_MECHATRONIKA</xsl:text></xsl:when><xsl:when test="$name = 'irp6o'"><xsl:text>,ROBOT_IRP6_ON_TRACK</xsl:text></xsl:when><xsl:when test="$name = 'irp6p'"><xsl:text>,ROBOT_IRP6_POSTUMENT</xsl:text></xsl:when><xsl:when test="$name = 'conveyor'"><xsl:text></xsl:text></xsl:when><xsl:otherwise><xsl:text>ROBOT_IRP6_NEWROBOT</xsl:text></xsl:otherwise></xsl:choose><xsl:text>
				);
				
				robot->get_controller_state(&amp;state);
}

edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>::~edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>()
{
	if (robot) {
		delete robot;
	}		
}

static edp_</xsl:text><xsl:value-of select="$name" /><xsl:text> *edp_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>;


extern "C" 
{ 
	void  on_combobox1_changed_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>(GtkComboBox *comboBox, gpointer userdata)  
	{
		ui_config_entry &amp; ChoseEntry = *(ui_config_entry *) userdata;
		GtkBuilder &amp; builder = (ChoseEntry.getBuilder());
		
		GtkScrolledWindow * scrolled = GTK_SCROLLED_WINDOW (gtk_builder_get_object(&amp;builder, "scrolledwindow_edp"));

		//if the child exists, destroy it
		if (gtk_bin_get_child(GTK_BIN(scrolled))!=NULL)
		{
			GtkWidget* child = gtk_bin_get_child(GTK_BIN(scrolled));
			gtk_widget_destroy(child);
		}

		gboolean isFile = 0;
		const gchar * windowName;
		gint choice;
		choice = gtk_combo_box_get_active (comboBox);

		switch (choice)
		{
		case 0: </xsl:text><xsl:if test="$motorsNo &gt; 0"><xsl:text>std::cout &lt;&lt; "Servo algorithm window chosen" &lt;&lt; std::endl; isFile = 1; windowName = "window_servo";</xsl:text></xsl:if><xsl:text> break;
		case 1: </xsl:text><xsl:if test="$motorsNo &gt; 0"><xsl:text>std::cout &lt;&lt; "Internal window chosen" &lt;&lt; std::endl; isFile = 1; windowName = "window_int";</xsl:text></xsl:if><xsl:text> break;
		case 2: </xsl:text><xsl:if test="$motorsNo &gt; 0"><xsl:text>std::cout &lt;&lt; "Increment window chosen" &lt;&lt; std::endl; isFile = 1; windowName = "window_inc";</xsl:text></xsl:if><xsl:text> break;
		case 3: </xsl:text><xsl:if test="$axis_xyz &gt; 0"><xsl:text>std::cout &lt;&lt; "XYZ Angle Axis window chosen" &lt;&lt; std::endl; isFile = 1; windowName = "window_axis_xyz";</xsl:text></xsl:if><xsl:text> break;
		case 4: </xsl:text><xsl:if test="$euler_xyz &gt; 0"><xsl:text>std::cout &lt;&lt; "XYZ Euler ZYZ window chosen" &lt;&lt; std::endl; isFile = 1; windowName = "window_euler_xyz";</xsl:text></xsl:if><xsl:text> break;
		case 5: </xsl:text><xsl:if test="$axis_ts &gt; 0"><xsl:text>std::cout &lt;&lt; "TS Angle Axis window chosen" &lt;&lt; std::endl; isFile = 1; windowName = "window_axis_ts";</xsl:text></xsl:if><xsl:text> break;
		case 6: </xsl:text><xsl:if test="$euler_ts &gt; 0"><xsl:text>std::cout &lt;&lt; "TS Euler ZYZ window chosen" &lt;&lt; std::endl; isFile = 1; windowName = "window_euler_ts";</xsl:text></xsl:if><xsl:text> break;
		default: std::cout &lt;&lt; "Something is not working properly!" &lt;&lt; std::endl;
		}
		
		if (isFile)
		{
	
			GtkWidget* chosenWindow = GTK_WIDGET (gtk_builder_get_object (&amp;builder, windowName));
			g_assert(chosenWindow);
			
			GtkWidget* windowWithoutParent = gtk_bin_get_child(GTK_BIN(chosenWindow));
			gtk_widget_unparent(windowWithoutParent);
			
			gtk_scrolled_window_add_with_viewport (scrolled, windowWithoutParent);
		}
		
	}	

	void ui_module_init(ui_config_entry &amp;entry) 
	{
		edp_</xsl:text><xsl:value-of select="$fullName" /><xsl:text> = new edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>(entry);
		fprintf(stderr, "module %s loaded\n", __FILE__);
	}

	void ui_module_unload(void) 
	{
		if (edp_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>) 
		{
			delete edp_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>;
		}
		fprintf(stderr, "module %s unloaded\n", __FILE__);
	}
}
</xsl:text>

	<xsl:if test="$motorsNo &gt; 0">
		<xsl:call-template name="irp6.servo.main.signals.cc" />
		<xsl:call-template name="irp6.int.main.signals.cc" />
		<xsl:call-template name="irp6.inc.main.signals.cc" />
	</xsl:if>
	<xsl:if test="$axis_xyz &gt; 0">	
		<xsl:call-template name="irp6.axis.xyz.main.signals.cc" />
	</xsl:if>
	<xsl:if test="$axis_ts &gt; 0">
		<xsl:call-template name="irp6.axis.ts.main.signals.cc" />
	</xsl:if>
	<xsl:if test="$euler_xyz &gt; 0">
		<xsl:call-template name="irp6.euler.xyz.main.signals.cc" />
	</xsl:if>
	<xsl:if test="$euler_ts &gt; 0">
		<xsl:call-template name="irp6.euler.ts.main.signals.cc" />
	</xsl:if>



</xsl:document>
<xsl:call-template name="irp6.edp.main.signals.h" />
</xsl:template>


<!-- signals handling file .cc-->
<xsl:template name="irp6.edp.main.signals.h" match="*[substring(name(),1,4)='irp6']">
<xsl:variable name="name" select="name"/>
<xsl:document method="text" href="../signals/edp_{$name}_uimodule.h">



<xsl:text>
#ifndef __EDP_</xsl:text><xsl:value-of select="$name" /><xsl:text>
#define __EDP_</xsl:text><xsl:value-of select="$name" /><xsl:text>

#include &lt;iostream&gt;
#include &lt;vector&gt;

#include &lt;gtk/gtkbuilder.h&gt;
#include &lt;gtk/gtk.h&gt;

#include "ui/ui_ecp_r_irp6_common.h"
#include "ui/ui_ecp_r_conveyor.h"

class edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>
{
	public:

		edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>(ui_config_entry &amp;entry);
		~edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>();
};
ui_</xsl:text><xsl:choose><xsl:when test="$name = 'conveyor'"><xsl:text>conveyor</xsl:text></xsl:when><xsl:otherwise><xsl:text>common</xsl:text></xsl:otherwise></xsl:choose><xsl:text>_robot * robot;
controller_state_t state;


#endif /* __EDP_</xsl:text><xsl:value-of select="$name" /><xsl:text> */
</xsl:text>

</xsl:document>
</xsl:template>


</xsl:stylesheet>
