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



<xsl:text>
#include &lt;iostream&gt;
#include &lt;stdio.h&gt;

//GTK libraries
#include &lt;gtk/gtk.h&gt;
#include &lt;glib.h&gt;

//UI model libraries
#include "ui_model.h"
#include "edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>_uimodule.h"

//mrrocpp UI constants
mrrocpp::lib::BYTE servo_alg_no[</xsl:text><xsl:value-of select="$motorsNo" /><xsl:text>];
mrrocpp::lib::BYTE servo_par_no[</xsl:text><xsl:value-of select="$motorsNo" /><xsl:text>];
	
gint servo_alg_no_tmp [</xsl:text><xsl:value-of select="$motorsNo" /><xsl:text>];
mrrocpp::lib::BYTE servo_alg_no_output[</xsl:text><xsl:value-of select="$motorsNo" /><xsl:text>];
gint servo_par_no_tmp [</xsl:text><xsl:value-of select="$motorsNo" /><xsl:text>];
mrrocpp::lib::BYTE servo_par_no_output[</xsl:text><xsl:value-of select="$motorsNo" /><xsl:text>];

mrrocpp::lib::BYTE model_no;
gint model_no_tmp;

char buf[32];
gchar buffer[500];
double tool_vector_a[</xsl:text><xsl:value-of select="$xyz_angle_axis_tool" /><xsl:text>];
double tool_vector_e[</xsl:text><xsl:value-of select="$xyz_euler_zyz_tool" /><xsl:text>];
double alfa, kx, ky, kz;
double wl; 
double l_eps = 0;
double </xsl:text><xsl:value-of select="$name" /><xsl:text>_current_pos_a[</xsl:text><xsl:value-of select="$xyz_angle_axis" /><xsl:text>]; // pozycja biezaca
double </xsl:text><xsl:value-of select="$name" /><xsl:text>_desired_pos_a[</xsl:text><xsl:value-of select="$xyz_angle_axis" /><xsl:text>]; // pozycja zadana
double </xsl:text><xsl:value-of select="$name" /><xsl:text>_current_pos_e[</xsl:text><xsl:value-of select="$xyz_euler_zyz" /><xsl:text>]; // pozycja biezaca
double </xsl:text><xsl:value-of select="$name" /><xsl:text>_desired_pos_e[</xsl:text><xsl:value-of select="$xyz_euler_zyz" /><xsl:text>]; // pozycja zadana
double </xsl:text><xsl:value-of select="$name" /><xsl:text>_current_pos[</xsl:text><xsl:value-of select="$motorsNo" /><xsl:text>]; // pozycja biezaca
double </xsl:text><xsl:value-of select="$name" /><xsl:text>_desired_pos[</xsl:text><xsl:value-of select="$motorsNo" /><xsl:text>]; // pozycja zadana



</xsl:text><xsl:choose><xsl:when test="$name = 'conveyor'"><xsl:text>#include "ui/ui_ecp_r_conveyor.h"</xsl:text></xsl:when><xsl:otherwise><xsl:text>#include "ui/ui_ecp_r_irp6_common.h"</xsl:text></xsl:otherwise></xsl:choose><xsl:text>

ui_</xsl:text><xsl:choose><xsl:when test="$name = 'conveyor'"><xsl:text>conveyor</xsl:text></xsl:when><xsl:otherwise><xsl:text>common</xsl:text></xsl:otherwise></xsl:choose><xsl:text>_robot * robot_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>;
mrrocpp::lib::controller_state_t state_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>;

//UI robot constructor
edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>::edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>()
{
				robot_</xsl:text><xsl:value-of select="$fullName" /><xsl:text> = new </xsl:text><xsl:choose><xsl:when test="$name = 'conveyor'"><xsl:text>ui_conveyor_robot</xsl:text></xsl:when><xsl:otherwise><xsl:text>ui_common_robot</xsl:text></xsl:otherwise></xsl:choose><xsl:text>(
				ui_model::instance().getConfigurator(),
				&amp;ui_model::instance().getEcpSr()
				</xsl:text><xsl:if test="$robotType != ''"><xsl:text>,mrrocpp::lib::</xsl:text></xsl:if><xsl:value-of select="$robotType" /><xsl:text>
				);

}

//UI robot desctructor
edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>::~edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>()
{
	if (robot_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>) {
		delete robot_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>;
	}		
}

static edp_</xsl:text><xsl:value-of select="$name" /><xsl:text> *edp_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>;


extern "C" 
{ 
	//callback signals for initializing the manual motion windows values
	void on_read_button_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_servo (GtkButton * button, gpointer userdata);
	void on_read_button_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_int (GtkButton * button, gpointer userdata);
	void on_read_button_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_inc (GtkButton * button, gpointer userdata);
	void on_read_button_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_xyz_angle_axis (GtkButton * button, gpointer userdata);
	void on_read_button_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_xyz_euler_zyz (GtkButton * button, gpointer userdata);
	void on_read_button_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_xyz_angle_axis_tool (GtkButton * button, gpointer userdata);
	void on_read_button_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_xyz_euler_zyz_tool (GtkButton * button, gpointer userdata);
	void on_read_button_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_kinematic (GtkButton * button, gpointer userdata);

	//handler for combo-box widget
	void  on_combobox1_changed_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>(GtkComboBox *comboBox, gpointer userdata)  
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
		if (state_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>.is_synchronised)
		{
			switch (choice)
			{
			case 0: </xsl:text><xsl:if test="$motorsNo &gt; 0"><xsl:text>std::cout &lt;&lt; "Internal joint window chosen" &lt;&lt; std::endl; isFile = 1; windowName = "window_int";</xsl:text></xsl:if><xsl:text> on_read_button_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_int (NULL, userdata); break;
			case 1: </xsl:text><xsl:if test="$motorsNo &gt; 0"><xsl:text>std::cout &lt;&lt; "Increment window chosen" &lt;&lt; std::endl; isFile = 1; windowName = "window_inc";</xsl:text></xsl:if><xsl:text> on_read_button_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_inc (NULL, userdata); break;
			case 2: </xsl:text><xsl:if test="$motorsNo &gt; 0"><xsl:text>std::cout &lt;&lt; "Servo algorithm window chosen" &lt;&lt; std::endl; isFile = 1; windowName = "window_servo";</xsl:text></xsl:if><xsl:text> on_read_button_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_servo (NULL, userdata); break;
			case 3: </xsl:text><xsl:if test="$xyz_angle_axis &gt; 0"><xsl:text>std::cout &lt;&lt; "XYZ Angle Axis window chosen" &lt;&lt; std::endl; isFile = 1; windowName = "window_xyz_angle_axis";</xsl:text></xsl:if><xsl:text> on_read_button_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_xyz_angle_axis (NULL, userdata); break;
			case 4: </xsl:text><xsl:if test="$xyz_euler_zyz &gt; 0"><xsl:text>std::cout &lt;&lt; "XYZ Euler ZYZ window chosen" &lt;&lt; std::endl; isFile = 1; windowName = "window_xyz_euler_zyz";</xsl:text></xsl:if><xsl:text> on_read_button_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_xyz_euler_zyz (NULL, userdata); break;
			case 5: </xsl:text><xsl:if test="$xyz_angle_axis_tool &gt; 0"><xsl:text>std::cout &lt;&lt; "XYZ Angle Axis tool window chosen" &lt;&lt; std::endl; isFile = 1; windowName = "window_xyz_angle_axis_tool";</xsl:text></xsl:if><xsl:text> on_read_button_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_xyz_angle_axis_tool (NULL, userdata); break;
			case 6: </xsl:text><xsl:if test="$xyz_euler_zyz_tool &gt; 0"><xsl:text>std::cout &lt;&lt; "XYZ Euler ZYZ tool window chosen" &lt;&lt; std::endl; isFile = 1; windowName = "window_xyz_euler_zyz_tool";</xsl:text></xsl:if><xsl:text> on_read_button_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_xyz_euler_zyz_tool (NULL, userdata); break;
			case 7: </xsl:text><xsl:if test="$kinematic &gt; 0"><xsl:text>std::cout &lt;&lt; "Kinematic model window chosen" &lt;&lt; std::endl; isFile = 1; windowName = "window_kinematic";</xsl:text></xsl:if><xsl:text> on_read_button_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_kinematic (NULL, userdata); break;
			default: std::cout &lt;&lt; "Synchronizing..." &lt;&lt; std::endl;
			}
		}
		//if robot is not synchronized
		else
		{
			switch (choice)
			{
			case 0: </xsl:text><xsl:if test="$motorsNo &gt; 0"><xsl:text>std::cout &lt;&lt; "Internal joint window chosen" &lt;&lt; std::endl; isFile = 1; windowName = "window_int";</xsl:text></xsl:if><xsl:text> on_read_button_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_int (NULL, userdata); break;
			case 1: </xsl:text><xsl:if test="$motorsNo &gt; 0"><xsl:text>std::cout &lt;&lt; "Increment window chosen" &lt;&lt; std::endl; isFile = 1; windowName = "window_inc";</xsl:text></xsl:if><xsl:text> on_read_button_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_inc (NULL, userdata); break;
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
	void  on_clicked_synchronize_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>(GtkButton * button, gpointer userdata)  
	{
		ui_config_entry &amp; comboEntry = *(ui_config_entry *) userdata;
		GtkBuilder &amp; builder = (comboEntry.getBuilder());
		
		robot_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>->get_controller_state (&amp;state_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>);

		if(!state_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>.is_synchronised) {
	   		GError *error = NULL;
	   		GThread * synchronization_thread_</xsl:text><xsl:value-of select="$fullName" /><xsl:text> = g_thread_create(ui_synchronize_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>, userdata, false, &amp;error);
			if (synchronization_thread_</xsl:text><xsl:value-of select="$fullName" /><xsl:text> == NULL) 
	     		{
				fprintf(stderr, "g_thread_create(): %s\n", error->message);
			}
		}

		robot_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>->get_controller_state (&amp;state_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>);

		if (state_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>.is_synchronised) {
			gtk_widget_set_sensitive( GTK_WIDGET(button), FALSE);
		    
			GtkComboBox * combo = GTK_COMBO_BOX (gtk_builder_get_object(&amp;builder, "combobox1"));

			</xsl:text><xsl:if test="$motorsNo &gt; 0"><xsl:text>gint counter_synch = 2;</xsl:text></xsl:if><xsl:text>
			</xsl:text><xsl:if test="$motorsNo &gt; 0"><xsl:text>gtk_combo_box_insert_text(combo, counter_synch, "Servo algorithm"); counter_synch++;</xsl:text></xsl:if><xsl:text>
			</xsl:text><xsl:if test="$xyz_angle_axis &gt; 0"><xsl:text>gtk_combo_box_insert_text(combo, counter_synch, "XYZ Angle Axis"); counter_synch++;</xsl:text></xsl:if><xsl:text>
			</xsl:text><xsl:if test="$xyz_euler_zyz &gt; 0"><xsl:text>gtk_combo_box_insert_text(combo, counter_synch, "XYZ Euler ZYZ"); counter_synch++;</xsl:text></xsl:if><xsl:text>
			</xsl:text><xsl:if test="$xyz_angle_axis_tool &gt; 0"><xsl:text>gtk_combo_box_insert_text(combo, counter_synch, "XYZ Angle Axis tool"); counter_synch++;</xsl:text></xsl:if><xsl:text>
			</xsl:text><xsl:if test="$xyz_euler_zyz_tool &gt; 0"><xsl:text>gtk_combo_box_insert_text(combo, counter_synch, "XYZ Euler ZYZ tool"); counter_synch++;</xsl:text></xsl:if><xsl:text>
			</xsl:text><xsl:if test="$kinematic &gt; 0"><xsl:text>gtk_combo_box_insert_text(combo, counter_synch, "Kinematic"); counter_synch++;</xsl:text></xsl:if><xsl:text>
		}
	}	

	//UI module initializing function
	void ui_module_init(ui_config_entry &amp;entry) 
	{
		edp_</xsl:text><xsl:value-of select="$fullName" /><xsl:text> = new edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>();
		fprintf(stderr, "module %s loaded\n", __FILE__);

		gint counter = 0;
		GtkBuilder &amp; builder = (entry.getBuilder());
		GtkButton * button = GTK_BUTTON (gtk_builder_get_object(&amp;builder, "button_synchronize"));
		if (state_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>.is_synchronised) gtk_widget_set_sensitive( GTK_WIDGET(button), FALSE);
		else
		{
			GtkComboBox * combo = GTK_COMBO_BOX (gtk_builder_get_object(&amp;builder, "combobox1"));
			
			</xsl:text><xsl:if test="$motorsNo &gt; 0"><xsl:text>gtk_combo_box_remove_text(combo, counter); gtk_combo_box_insert_text(combo, counter, "Internal joint"); counter++; gtk_combo_box_insert_text(combo, counter, "Increment"); counter++;</xsl:text></xsl:if><xsl:text>
		}
	}

	//executed when GTK main window is being closed. 
	void ui_module_unload(void) 
	{
		if (edp_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>) 
		{
			delete edp_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>;
		}
		fprintf(stderr, "module %s unloaded\n", __FILE__);
	}
}

//function used by the synchronization thread - used to synchronize the robot
void *ui_synchronize_</xsl:text><xsl:value-of select="$fullName" /><xsl:text> (gpointer userdata)
{
	ui_config_entry &amp; comboEntry = *(ui_config_entry *) userdata;
	GtkBuilder &amp; builder = (comboEntry.getBuilder());
	gint counter = 0;

	robot_</xsl:text><xsl:value-of select="$fullName" /><xsl:choose><xsl:when test="$name != 'conveyor'"><xsl:text>->ecp</xsl:text></xsl:when></xsl:choose><xsl:text>->synchronise();
    
	GtkComboBox * combo = GTK_COMBO_BOX (gtk_builder_get_object(&amp;builder, "combobox1"));

	</xsl:text><xsl:if test="$motorsNo &gt; 0"><xsl:text>counter = 2;</xsl:text></xsl:if><xsl:text>
	</xsl:text><xsl:if test="$motorsNo &gt; 0"><xsl:text>gtk_combo_box_insert_text(combo, counter, "Servo algorithm"); counter++;</xsl:text></xsl:if><xsl:text>
	</xsl:text><xsl:if test="$xyz_angle_axis &gt; 0"><xsl:text>gtk_combo_box_insert_text(combo, counter, "XYZ Angle Axis"); counter++;</xsl:text></xsl:if><xsl:text>
	</xsl:text><xsl:if test="$xyz_euler_zyz &gt; 0"><xsl:text>gtk_combo_box_insert_text(combo, counter, "XYZ Euler ZYZ"); counter++;</xsl:text></xsl:if><xsl:text>
	</xsl:text><xsl:if test="$xyz_angle_axis_tool &gt; 0"><xsl:text>gtk_combo_box_insert_text(combo, counter, "XYZ Angle Axis tool"); counter++;</xsl:text></xsl:if><xsl:text>
	</xsl:text><xsl:if test="$xyz_euler_zyz_tool &gt; 0"><xsl:text>gtk_combo_box_insert_text(combo, counter, "XYZ Euler ZYZ tool"); counter++;</xsl:text></xsl:if><xsl:text>
	</xsl:text><xsl:if test="$kinematic &gt; 0"><xsl:text>gtk_combo_box_insert_text(combo, counter, "Kinematic"); counter++;</xsl:text></xsl:if><xsl:text>
	return NULL;
}

</xsl:text>

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

		edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>();
		~edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>();
};

void *ui_synchronize_</xsl:text><xsl:value-of select="$fullName" /><xsl:text> (gpointer userdata);

#endif /* __EDP_</xsl:text><xsl:value-of select="$name" /><xsl:text> */
</xsl:text>

</xsl:document>
</xsl:template>


</xsl:stylesheet>
