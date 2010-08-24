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
<xsl:variable name="xyz_angle_axis" select="xyz_angle_axis"/>
<xsl:variable name="xyz_angle_axis_tool" select="xyz_angle_axis_tool"/>
<xsl:variable name="xyz_euler_zyz" select="xyz_euler_zyz"/>
<xsl:variable name="xyz_euler_zyz_tool" select="xyz_euler_zyz_tool"/>
<xsl:variable name="kinematic" select="kinematic"/>
<xsl:variable name="robotType" select="robotType"/>
<xsl:document method="text" href="../signals/edp_{$name}_uimodule.cc">



#include &lt;iostream&gt;
#include &lt;cstdio&gt;
#include &lt;math.h&gt;

//GTK libraries
#include &lt;gtk/gtk.h&gt;
#include &lt;glib.h&gt;

//UI model libraries
#include "ui_model.h"
#include "edp_<xsl:value-of select="$name" />_uimodule.h"

//mrrocpp UI constants
static uint8_t servo_alg_no[<xsl:value-of select="$motorsNo" />];
static uint8_t servo_par_no[<xsl:value-of select="$motorsNo" />];
	
static gint servo_alg_no_tmp [<xsl:value-of select="$motorsNo" />];
static uint8_t servo_alg_no_output[<xsl:value-of select="$motorsNo" />];
static gint servo_par_no_tmp [<xsl:value-of select="$motorsNo" />];
static uint8_t servo_par_no_output[<xsl:value-of select="$motorsNo" />];

static uint8_t model_no;
static gint model_no_tmp;

static char buf[32];
static gchar buffer[500];
<xsl:choose><xsl:when test="$name = 'conveyor'"></xsl:when><xsl:otherwise>
static double alfa;
static double l_eps = 0;
static double <xsl:value-of select="$name" />_current_pos_a[<xsl:value-of select="$xyz_angle_axis" />]; // pozycja biezaca
static double <xsl:value-of select="$name" />_desired_pos_a[<xsl:value-of select="$xyz_angle_axis" />]; // pozycja zadana
static double <xsl:value-of select="$name" />_current_pos_e[<xsl:value-of select="$xyz_euler_zyz" />]; // pozycja biezaca
static double <xsl:value-of select="$name" />_desired_pos_e[<xsl:value-of select="$xyz_euler_zyz" />]; // pozycja zadana
</xsl:otherwise></xsl:choose>
static double <xsl:value-of select="$name" />_current_pos[<xsl:value-of select="$motorsNo" />]; // pozycja biezaca
static double <xsl:value-of select="$name" />_desired_pos[<xsl:value-of select="$motorsNo" />]; // pozycja zadana



<xsl:choose><xsl:when test="$name = 'conveyor'">#include "ui/ui_ecp_r_tfg_and_conv.h"</xsl:when><xsl:otherwise>#include "ui/ui_ecp_r_irp6_common.h"</xsl:otherwise></xsl:choose>

ui_<xsl:choose><xsl:when test="$name = 'conveyor'">tfg_and_conv</xsl:when><xsl:otherwise>irp6_common</xsl:otherwise></xsl:choose>_robot * robot_<xsl:value-of select="$fullName" />;
mrrocpp::lib::controller_state_t state_<xsl:value-of select="$fullName" />;

//UI robot constructor
edp_<xsl:value-of select="$name" />::edp_<xsl:value-of select="$name" />()
{
				robot_<xsl:value-of select="$fullName" /> = new <xsl:choose><xsl:when test="$name = 'conveyor'">ui_tfg_and_conv_robot</xsl:when><xsl:otherwise>ui_irp6_common_robot</xsl:otherwise></xsl:choose>(
				ui_model::instance().getConfigurator(),
				ui_model::instance().getEcpSr()
				<xsl:if test="$robotType != ''">,mrrocpp::lib::</xsl:if><xsl:value-of select="$robotType" />
				);

				robot_<xsl:value-of select="$fullName" />->get_controller_state(state_<xsl:value-of select="$fullName" />);
}

//UI robot desctructor
edp_<xsl:value-of select="$name" />::~edp_<xsl:value-of select="$name" />()
{
	if (robot_<xsl:value-of select="$fullName" />) {
		delete robot_<xsl:value-of select="$fullName" />;
	}		
}

static edp_<xsl:value-of select="$name" /> *edp_<xsl:value-of select="$fullName" />;


extern "C" 
{ 
	//callback signals for initializing the manual motion windows values
	void on_read_button_clicked_<xsl:value-of select="$fullName" />_servo (GtkButton * button, gpointer userdata);
	void on_read_button_clicked_<xsl:value-of select="$fullName" />_int (GtkButton * button, gpointer userdata);
	void on_read_button_clicked_<xsl:value-of select="$fullName" />_inc (GtkButton * button, gpointer userdata);
	void on_read_button_clicked_<xsl:value-of select="$fullName" />_xyz_angle_axis (GtkButton * button, gpointer userdata);
	void on_read_button_clicked_<xsl:value-of select="$fullName" />_xyz_euler_zyz (GtkButton * button, gpointer userdata);
	void on_read_button_clicked_<xsl:value-of select="$fullName" />_xyz_angle_axis_tool (GtkButton * button, gpointer userdata);
	void on_read_button_clicked_<xsl:value-of select="$fullName" />_xyz_euler_zyz_tool (GtkButton * button, gpointer userdata);
	void on_read_button_clicked_<xsl:value-of select="$fullName" />_kinematic (GtkButton * button, gpointer userdata);

	//handler for combo-box widget
	void  on_combobox1_changed_<xsl:value-of select="$fullName" />(GtkComboBox *comboBox, gpointer userdata)  
	{
		ui_config_entry &amp; ChoseEntry = *(ui_config_entry *) userdata;
		GtkBuilder &amp; builder = (ChoseEntry.getBuilder());
		
		GtkScrolledWindow * scrolled = GTK_SCROLLED_WINDOW (gtk_builder_get_object(&amp;builder, "scrolledwindow_edp"));

		//if the child exists, destroy it
		if (gtk_bin_get_child(GTK_BIN(scrolled))!=NULL)
		{
			GtkWidget* child = gtk_bin_get_child(GTK_BIN(scrolled));
			GtkContainer * container = GTK_CONTAINER (scrolled);
			GObject * object = G_OBJECT(child);
			g_object_ref(object);			
			gtk_container_remove(container, child);
		}

		gboolean isFile = 0;
		const gchar * windowName;
		gint choice;
		choice = gtk_combo_box_get_active (comboBox); 

		//which window has been chosen
		if (state_<xsl:value-of select="$fullName" />.is_synchronised)
		{
			switch (choice)
			{
			case 0: <xsl:if test="$motorsNo &gt; 0">std::cout &lt;&lt; "Internal joint window chosen" &lt;&lt; std::endl; isFile = 1; windowName = "window_int";</xsl:if> on_read_button_clicked_<xsl:value-of select="$fullName" />_int (NULL, userdata); break;
			case 1: <xsl:if test="$motorsNo &gt; 0">std::cout &lt;&lt; "Increment window chosen" &lt;&lt; std::endl; isFile = 1; windowName = "window_inc";</xsl:if> on_read_button_clicked_<xsl:value-of select="$fullName" />_inc (NULL, userdata); break;
			case 2: <xsl:if test="$motorsNo &gt; 0">std::cout &lt;&lt; "Servo algorithm window chosen" &lt;&lt; std::endl; isFile = 1; windowName = "window_servo";</xsl:if> on_read_button_clicked_<xsl:value-of select="$fullName" />_servo (NULL, userdata); break;
			case 3: <xsl:if test="$xyz_angle_axis &gt; 0">std::cout &lt;&lt; "XYZ Angle Axis window chosen" &lt;&lt; std::endl; isFile = 1; windowName = "window_xyz_angle_axis";</xsl:if> on_read_button_clicked_<xsl:value-of select="$fullName" />_xyz_angle_axis (NULL, userdata); break;
			case 4: <xsl:if test="$xyz_euler_zyz &gt; 0">std::cout &lt;&lt; "XYZ Euler ZYZ window chosen" &lt;&lt; std::endl; isFile = 1; windowName = "window_xyz_euler_zyz";</xsl:if> on_read_button_clicked_<xsl:value-of select="$fullName" />_xyz_euler_zyz (NULL, userdata); break;
			case 5: <xsl:if test="$xyz_angle_axis_tool &gt; 0">std::cout &lt;&lt; "XYZ Angle Axis tool window chosen" &lt;&lt; std::endl; isFile = 1; windowName = "window_xyz_angle_axis_tool";</xsl:if> on_read_button_clicked_<xsl:value-of select="$fullName" />_xyz_angle_axis_tool (NULL, userdata); break;
			case 6: <xsl:if test="$xyz_euler_zyz_tool &gt; 0">std::cout &lt;&lt; "XYZ Euler ZYZ tool window chosen" &lt;&lt; std::endl; isFile = 1; windowName = "window_xyz_euler_zyz_tool";</xsl:if> on_read_button_clicked_<xsl:value-of select="$fullName" />_xyz_euler_zyz_tool (NULL, userdata); break;
			case 7: <xsl:if test="$kinematic &gt; 0">std::cout &lt;&lt; "Kinematic model window chosen" &lt;&lt; std::endl; isFile = 1; windowName = "window_kinematic";</xsl:if> on_read_button_clicked_<xsl:value-of select="$fullName" />_kinematic (NULL, userdata); break;
			default: std::cout &lt;&lt; "Synchronizing..." &lt;&lt; std::endl;
			}
		}
		//if robot is not synchronized
		else
		{
			switch (choice)
			{
			case 0: <xsl:if test="$motorsNo &gt; 0">std::cout &lt;&lt; "Internal joint window chosen" &lt;&lt; std::endl; isFile = 1; windowName = "window_int";</xsl:if> on_read_button_clicked_<xsl:value-of select="$fullName" />_int (NULL, userdata); break;
			case 1: <xsl:if test="$motorsNo &gt; 0">std::cout &lt;&lt; "Increment window chosen" &lt;&lt; std::endl; isFile = 1; windowName = "window_inc";</xsl:if> on_read_button_clicked_<xsl:value-of select="$fullName" />_inc (NULL, userdata); break;
			default: ;
			}
		}
		
		if (isFile)
		{	
			GtkWidget* chosenWindow = GTK_WIDGET (gtk_builder_get_object (&amp;builder, windowName));
			g_assert(chosenWindow);
			
			GtkWidget* windowWithoutParent = gtk_bin_get_child(GTK_BIN(chosenWindow));
			gtk_widget_unparent(windowWithoutParent);
			
			//add specific manual motion window to the main window for the robot
			gtk_scrolled_window_add_with_viewport (scrolled, windowWithoutParent);
		}
		
	}	
	
	//handler for Synchronize button
	void  on_clicked_synchronize_<xsl:value-of select="$fullName" />(GtkButton * button, gpointer userdata)  
	{
		ui_config_entry &amp; comboEntry = *(ui_config_entry *) userdata;
		GtkBuilder &amp; builder = (comboEntry.getBuilder());
		
		if(!state_<xsl:value-of select="$fullName" />.is_synchronised) {
	   		GError *error = NULL;
	   		GThread * synchronization_thread_<xsl:value-of select="$fullName" /> = g_thread_create(ui_synchronize_<xsl:value-of select="$fullName" />, userdata, false, &amp;error);
			if (synchronization_thread_<xsl:value-of select="$fullName" /> == NULL) 
	     		{
				fprintf(stderr, "g_thread_create(): %s\n", error->message);
			}
		}

		try {
			robot_<xsl:value-of select="$fullName" />->get_controller_state (state_<xsl:value-of select="$fullName" />);

			// TODO: this should be checked in synchronization thread
			if (state_<xsl:value-of select="$fullName" />.is_synchronised) {
				gtk_widget_set_sensitive( GTK_WIDGET(button), FALSE);
			    
				GtkComboBox * combo = GTK_COMBO_BOX (gtk_builder_get_object(&amp;builder, "combobox1"));
	
				<xsl:if test="$motorsNo &gt; 0">gint counter_synch = 2;</xsl:if>
				<xsl:if test="$motorsNo &gt; 0">gtk_combo_box_insert_text(combo, counter_synch, "Servo algorithm"); counter_synch++;</xsl:if>
				<xsl:if test="$xyz_angle_axis &gt; 0">gtk_combo_box_insert_text(combo, counter_synch, "XYZ Angle Axis"); counter_synch++;</xsl:if>
				<xsl:if test="$xyz_euler_zyz &gt; 0">gtk_combo_box_insert_text(combo, counter_synch, "XYZ Euler ZYZ"); counter_synch++;</xsl:if>
				<xsl:if test="$xyz_angle_axis_tool &gt; 0">gtk_combo_box_insert_text(combo, counter_synch, "XYZ Angle Axis tool"); counter_synch++;</xsl:if>
				<xsl:if test="$xyz_euler_zyz_tool &gt; 0">gtk_combo_box_insert_text(combo, counter_synch, "XYZ Euler ZYZ tool"); counter_synch++;</xsl:if>
				<xsl:if test="$kinematic &gt; 0">gtk_combo_box_insert_text(combo, counter_synch, "Kinematic"); counter_synch++;</xsl:if>
			}
		}
		<xsl:call-template name="catch" />
	}	

	//UI module initializing function
	void ui_module_init(ui_config_entry &amp;entry) 
	{
		edp_<xsl:value-of select="$fullName" /> = new edp_<xsl:value-of select="$name" />();
		fprintf(stderr, "module %s loaded\n", __FILE__);

		GtkBuilder &amp; builder = (entry.getBuilder());

		GtkButton * button = GTK_BUTTON (gtk_builder_get_object(&amp;builder, "button_synchronize"));

		GtkComboBox * combo = GTK_COMBO_BOX (gtk_builder_get_object(&amp;builder, "combobox1"));
		gint counter = 0;
		<xsl:if test="$motorsNo &gt; 0">gtk_combo_box_remove_text(combo, counter);
		gtk_combo_box_insert_text(combo, counter++, "Internal joint");
		gtk_combo_box_insert_text(combo, counter++, "Increment");</xsl:if>

		if (state_<xsl:value-of select="$fullName" />.is_synchronised) {
			gtk_widget_set_sensitive(GTK_WIDGET(button), FALSE);
			
			<xsl:if test="$motorsNo &gt; 0">gtk_combo_box_insert_text(combo, counter++, "Servo algorithm");</xsl:if>
			<xsl:if test="$xyz_angle_axis &gt; 0">gtk_combo_box_insert_text(combo, counter++, "XYZ Angle Axis");</xsl:if>
			<xsl:if test="$xyz_euler_zyz &gt; 0">gtk_combo_box_insert_text(combo, counter++, "XYZ Euler ZYZ");</xsl:if>
			<xsl:if test="$xyz_angle_axis_tool &gt; 0">gtk_combo_box_insert_text(combo, counter++, "XYZ Angle Axis tool");</xsl:if>
			<xsl:if test="$xyz_euler_zyz_tool &gt; 0">gtk_combo_box_insert_text(combo, counter++, "XYZ Euler ZYZ tool");</xsl:if>
			<xsl:if test="$kinematic &gt; 0">gtk_combo_box_insert_text(combo, counter++, "Kinematic");</xsl:if>
		} else {
			gtk_widget_set_sensitive(GTK_WIDGET(button), TRUE);
		}
	}

	//executed when GTK main window is being closed. 
	void ui_module_unload(void) 
	{
		if (edp_<xsl:value-of select="$fullName" />) 
		{
			delete edp_<xsl:value-of select="$fullName" />;
		}
		fprintf(stderr, "module %s unloaded\n", __FILE__);
	}
}

//function used by the synchronization thread - used to synchronize the robot
void *ui_synchronize_<xsl:value-of select="$fullName" /> (gpointer userdata)
{
	ui_config_entry &amp; comboEntry = *(ui_config_entry *) userdata;
	GtkBuilder &amp; builder = (comboEntry.getBuilder());
	gint counter = 0;

	try {
		robot_<xsl:value-of select="$fullName" />->ecp->synchronise();
	
	GtkComboBox * combo = GTK_COMBO_BOX (gtk_builder_get_object(&amp;builder, "combobox1"));

	<xsl:if test="$motorsNo &gt; 0">counter = 2;</xsl:if>
	<xsl:if test="$motorsNo &gt; 0">gtk_combo_box_insert_text(combo, counter, "Servo algorithm"); counter++;</xsl:if>
	<xsl:if test="$xyz_angle_axis &gt; 0">gtk_combo_box_insert_text(combo, counter, "XYZ Angle Axis"); counter++;</xsl:if>
	<xsl:if test="$xyz_euler_zyz &gt; 0">gtk_combo_box_insert_text(combo, counter, "XYZ Euler ZYZ"); counter++;</xsl:if>
	<xsl:if test="$xyz_angle_axis_tool &gt; 0">gtk_combo_box_insert_text(combo, counter, "XYZ Angle Axis tool"); counter++;</xsl:if>
	<xsl:if test="$xyz_euler_zyz_tool &gt; 0">gtk_combo_box_insert_text(combo, counter, "XYZ Euler ZYZ tool"); counter++;</xsl:if>
	<xsl:if test="$kinematic &gt; 0">gtk_combo_box_insert_text(combo, counter, "Kinematic"); counter++;</xsl:if>
	
	}
	<xsl:call-template name="catch" />
    
	return NULL;
}



	<xsl:if test="$motorsNo &gt; 0">
		<xsl:call-template name="irp6.servo.main.signals.cc" />
		<xsl:call-template name="irp6.int.main.signals.cc" />
		<xsl:call-template name="irp6.inc.main.signals.cc" />
	</xsl:if>
	<xsl:if test="$xyz_angle_axis &gt; 0">	
		<xsl:call-template name="irp6.xyz_angle_axis.main.signals.cc" />
	</xsl:if>
	<xsl:if test="$xyz_angle_axis_tool &gt; 0">
		<xsl:call-template name="irp6.xyz_angle_axis_tool.main.signals.cc" />
	</xsl:if>
	<xsl:if test="$xyz_euler_zyz &gt; 0">
		<xsl:call-template name="irp6.xyz_euler_zyz.main.signals.cc" />
	</xsl:if>
	<xsl:if test="$xyz_euler_zyz_tool &gt; 0">
		<xsl:call-template name="irp6.xyz_euler_zyz_tool.main.signals.cc" />
	</xsl:if>
	<xsl:if test="$kinematic &gt; 0">
		<xsl:call-template name="irp6.kinematic.main.signals.cc" />
	</xsl:if>


</xsl:document>
<xsl:call-template name="irp6.edp.main.signals.h" />
</xsl:template>


<!-- signals handling file .cc-->
<xsl:template name="irp6.edp.main.signals.h" match="*[substring(name(),1,4)='irp6']">
<xsl:variable name="name" select="name"/>
<xsl:variable name="fullName" select="fullName"/>
<xsl:document method="text" href="../signals/edp_{$name}_uimodule.h">




#ifndef __EDP_<xsl:value-of select="$name" />
#define __EDP_<xsl:value-of select="$name" />

#include &lt;iostream&gt;
#include &lt;vector&gt;

#include &lt;gtk/gtkbuilder.h&gt;
#include &lt;gtk/gtk.h&gt;

<xsl:choose><xsl:when test="$name = 'conveyor'">#include "ui/ui_ecp_r_tfg_and_conv.h"</xsl:when><xsl:otherwise>#include "ui/ui_ecp_r_irp6_common.h"</xsl:otherwise></xsl:choose>

class edp_<xsl:value-of select="$name" />
{
	public:

		edp_<xsl:value-of select="$name" />();
		~edp_<xsl:value-of select="$name" />();
};

void *ui_synchronize_<xsl:value-of select="$fullName" /> (gpointer userdata);

#endif /* __EDP_<xsl:value-of select="$name" /> */


</xsl:document>
</xsl:template>


</xsl:stylesheet>
