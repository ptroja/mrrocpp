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

//UI robot constructor
edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>::edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>(ui_config_entry &amp;entry)
{
				robot_</xsl:text><xsl:value-of select="$fullName" /><xsl:text> = new </xsl:text><xsl:choose><xsl:when test="$name = 'conveyor'"><xsl:text>ui_conveyor_robot</xsl:text></xsl:when><xsl:otherwise><xsl:text>ui_common_robot</xsl:text></xsl:otherwise></xsl:choose><xsl:text>(
				ui_model::instance().getConfigurator(),
				&amp;ui_model::instance().getEcpSr()
				</xsl:text><xsl:choose><xsl:when test="$name = 'irp6m'"><xsl:text>,mrrocpp::lib::ROBOT_IRP6_MECHATRONIKA</xsl:text></xsl:when><xsl:when test="$name = 'irp6o'"><xsl:text>,mrrocpp::lib::ROBOT_IRP6_ON_TRACK</xsl:text></xsl:when><xsl:when test="$name = 'irp6p'"><xsl:text>,mrrocpp::lib::ROBOT_IRP6_POSTUMENT</xsl:text></xsl:when><xsl:when test="$name = 'conveyor'"><xsl:text></xsl:text></xsl:when><xsl:otherwise><xsl:text>ROBOT_IRP6_NEWROBOT</xsl:text></xsl:otherwise></xsl:choose><xsl:text>
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
	void on_read_button_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_axis_xyz (GtkButton * button, gpointer userdata);
	void on_read_button_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_euler_xyz (GtkButton * button, gpointer userdata);
	void on_read_button_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_axis_ts (GtkButton * button, gpointer userdata);
	void on_read_button_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_euler_ts (GtkButton * button, gpointer userdata);

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
			case 0: </xsl:text><xsl:if test="$motorsNo &gt; 0"><xsl:text>std::cout &lt;&lt; "Internal joint window chosen" &lt;&lt; std::endl; isFile = 1; windowName = "window_int";</xsl:text></xsl:if><xsl:text> on_read_button_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_int (button, userdata); break;
			case 1: </xsl:text><xsl:if test="$motorsNo &gt; 0"><xsl:text>std::cout &lt;&lt; "Increment window chosen" &lt;&lt; std::endl; isFile = 1; windowName = "window_inc";</xsl:text></xsl:if><xsl:text> on_read_button_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_inc (button, userdata); break;
			case 2: </xsl:text><xsl:if test="$motorsNo &gt; 0"><xsl:text>std::cout &lt;&lt; "Servo algorithm window chosen" &lt;&lt; std::endl; isFile = 1; windowName = "window_servo";</xsl:text></xsl:if><xsl:text> on_read_button_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_servo (button, userdata); break;
			case 3: </xsl:text><xsl:if test="$axis_xyz &gt; 0"><xsl:text>std::cout &lt;&lt; "XYZ Angle Axis window chosen" &lt;&lt; std::endl; isFile = 1; windowName = "window_axis_xyz";</xsl:text></xsl:if><xsl:text> on_read_button_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_axis_xyz (button, userdata); break;
			case 4: </xsl:text><xsl:if test="$euler_xyz &gt; 0"><xsl:text>std::cout &lt;&lt; "XYZ Euler ZYZ window chosen" &lt;&lt; std::endl; isFile = 1; windowName = "window_euler_xyz";</xsl:text></xsl:if><xsl:text> on_read_button_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_euler_xyz (button, userdata); break;
			case 5: </xsl:text><xsl:if test="$axis_ts &gt; 0"><xsl:text>std::cout &lt;&lt; "TS Angle Axis window chosen" &lt;&lt; std::endl; isFile = 1; windowName = "window_axis_ts";</xsl:text></xsl:if><xsl:text> on_read_button_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_axis_ts (button, userdata); break;
			case 6: </xsl:text><xsl:if test="$euler_ts &gt; 0"><xsl:text>std::cout &lt;&lt; "TS Euler ZYZ window chosen" &lt;&lt; std::endl; isFile = 1; windowName = "window_euler_ts";</xsl:text></xsl:if><xsl:text> on_read_button_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_euler_ts (button, userdata); break;
			default: std::cout &lt;&lt; "Synchronizing..." &lt;&lt; std::endl;
			}
		}
		//if robot is not synchronized
		else
		{
			switch (choice)
			{
			case 0: </xsl:text><xsl:if test="$motorsNo &gt; 0"><xsl:text>std::cout &lt;&lt; "Internal joint window chosen" &lt;&lt; std::endl; isFile = 1; windowName = "window_int";</xsl:text></xsl:if><xsl:text> on_read_button_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_int (button, userdata); break;
			case 1: </xsl:text><xsl:if test="$motorsNo &gt; 0"><xsl:text>std::cout &lt;&lt; "Increment window chosen" &lt;&lt; std::endl; isFile = 1; windowName = "window_inc";</xsl:text></xsl:if><xsl:text> on_read_button_clicked_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>_inc (button, userdata); break;
			case 2: break;
			case 3: break;
			case 4: break;
			case 5: break;
			case 6: break;
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
		gint counter_synch;
		
		robot_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>->get_controller_state (&amp;state_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>);
	        if(!state_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>.is_synchronised) {
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

					</xsl:text><xsl:if test="$motorsNo &gt; 0"><xsl:text>counter_synch = 2;</xsl:text></xsl:if><xsl:text>
					</xsl:text><xsl:if test="$motorsNo &gt; 0"><xsl:text>gtk_combo_box_insert_text(combo, counter_synch, "Servo algorithm"); counter_synch++;</xsl:text></xsl:if><xsl:text>
					</xsl:text><xsl:if test="$axis_xyz &gt; 0"><xsl:text>gtk_combo_box_insert_text(combo, counter_synch, "XYZ Angle Axis"); counter_synch++;</xsl:text></xsl:if><xsl:text>
					</xsl:text><xsl:if test="$euler_xyz &gt; 0"><xsl:text>gtk_combo_box_insert_text(combo, counter_synch, "XYZ Euler ZYZ"); counter_synch++;</xsl:text></xsl:if><xsl:text>
					</xsl:text><xsl:if test="$axis_ts &gt; 0"><xsl:text>gtk_combo_box_insert_text(combo, counter_synch, "TS Angle Axis"); counter_synch++;</xsl:text></xsl:if><xsl:text>
					</xsl:text><xsl:if test="$euler_ts &gt; 0"><xsl:text>gtk_combo_box_insert_text(combo, counter_synch, "TS Euler ZYZ"); counter_synch++;</xsl:text></xsl:if><xsl:text>

	        }
	}	

	//UI module initializing function
	void ui_module_init(ui_config_entry &amp;entry) 
	{
		edp_</xsl:text><xsl:value-of select="$fullName" /><xsl:text> = new edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>(entry);
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
	</xsl:text><xsl:if test="$axis_xyz &gt; 0"><xsl:text>gtk_combo_box_insert_text(combo, counter, "XYZ Angle Axis"); counter++;</xsl:text></xsl:if><xsl:text>
	</xsl:text><xsl:if test="$euler_xyz &gt; 0"><xsl:text>gtk_combo_box_insert_text(combo, counter, "XYZ Euler ZYZ"); counter++;</xsl:text></xsl:if><xsl:text>
	</xsl:text><xsl:if test="$axis_ts &gt; 0"><xsl:text>gtk_combo_box_insert_text(combo, counter, "TS Angle Axis"); counter++;</xsl:text></xsl:if><xsl:text>
	</xsl:text><xsl:if test="$euler_ts &gt; 0"><xsl:text>gtk_combo_box_insert_text(combo, counter, "TS Euler ZYZ"); counter++;</xsl:text></xsl:if><xsl:text>
	return NULL;
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

		edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>(ui_config_entry &amp;entry);
		~edp_</xsl:text><xsl:value-of select="$name" /><xsl:text>();
};
ui_</xsl:text><xsl:choose><xsl:when test="$name = 'conveyor'"><xsl:text>conveyor</xsl:text></xsl:when><xsl:otherwise><xsl:text>common</xsl:text></xsl:otherwise></xsl:choose><xsl:text>_robot * robot_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>;
mrrocpp::lib::controller_state_t state_</xsl:text><xsl:value-of select="$fullName" /><xsl:text>;
GError *error = NULL;
void *ui_synchronize_</xsl:text><xsl:value-of select="$fullName" /><xsl:text> (gpointer userdata);
GtkButton* button;

#endif /* __EDP_</xsl:text><xsl:value-of select="$name" /><xsl:text> */
</xsl:text>

</xsl:document>
</xsl:template>


</xsl:stylesheet>
