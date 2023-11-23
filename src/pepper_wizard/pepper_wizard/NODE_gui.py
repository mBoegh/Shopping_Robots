"""
TO DO:
 - Make UI with visualisation of detected mental command and its power (Eg. 'Lift' or 'Drop' and power 0-100)
"""

from pepper_wizard.PEPLIB import JSON_Handler        
        
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from customtkinter import *


class variables:
    
    def __init__(self):
        pass


class Gui(Node):
    """
    This is the gui node of the EXONET ROS2 network.
    Takes argument(s):
     - log_debug (Bool for toggling logging of severity level 'debug', 'info' and 'warn'. Severity level 'error' and 'fatal' is always logged.)
    """

    def __init__(self, timer_period, log_debug):

        # Initialising variables
        self.TIMER_PERIOD = timer_period
        self.LOG_DEBUG = log_debug

        # Initialising the 'Node' class, from which this class is inheriting, with argument 'node_name'
        super().__init__('gui')

        self.app = MainW(None)

        # Initialising a publisher to the topic 'EEG_toggle'.
        # On this topic is published data of type std_msgs.msg.Bool which is imported as Bool.
        # The '10' argument is some Quality of Service parameter (QoS).
        self.command_publisher = self.create_publisher(String, 'Command', 10)
        self.command_publisher  # prevent unused variable warning

        # The below functions are what actually does the updating of the window
        # We do also have a function called "mainloop()", but the program will halt
        # when it gets to "mainloop()", so only use it if you plan on destroying the window
        # when updating it, by making a new window
        self.app.update_idletasks()
        self.app.update()


class MainW(CTk):
    def __init__(self, parent):
        super().__init__(parent)
        self.geometry("1200x800")
        self.parent = parent
        self.title("Pepper_Wizard GUI")
        self.mainWidgets()

        self.speech_menu_window = None
        self.animations_menu_window = None
        self.system_menu_window = None

    def mainWidgets(self):
        """Calls and arranges all frames needed in the main window"""
        self.manual_frame = ManualControl(self)

        self.manual_frame.grid(row= 1, column= 0, pady= 20, padx= 60)

        self.debug_button = CTkButton(master=self.manual_frame, text="Speech", command=self.open_speech_menu)
        self.debug_button.grid(row= 0, column= 0, padx= 10, pady= 5)

        self.debug_button = CTkButton(master=self.manual_frame, text="Animations", command=self.open_animations_menu)
        self.debug_button.grid(row= 0, column= 1, padx= 10, pady= 5)

        self.debug_button = CTkButton(master=self.manual_frame, text="System", command=self.open_system_menu)
        self.debug_button.grid(row= 0, column= 2, padx= 10, pady= 5)


    def open_speech_menu(self):
        """First chekcs if the debug menu exists (is open), and if it isnt
        Then it creates the window. Or if it does exist, 
        then it lifts the window and sets the focus to it"""
        if self.speech_menu_window is None or not self.speech_menu_window.winfo_exists():
            self.speech_menu_window = Speech_Menu()
        else:
            self.speech_menu_window.focus()
            self.speech_menu_window.lift()


    def open_animations_menu(self):
        """First chekcs if the debug menu exists (is open), and if it isnt
        Then it creates the window. Or if it does exist, 
        then it lifts the window and sets the focus to it"""
        if self.animations_menu_window is None or not self.animations_menu_window.winfo_exists():
            self.animations_menu_window = Animations_Menu()
        else:
            self.animations_menu_window.focus()
            self.animations_menu_window.lift()

     
    def open_system_menu(self):
        """First chekcs if the debug menu exists (is open), and if it isnt
        Then it creates the window. Or if it does exist, 
        then it lifts the window and sets the focus to it"""
        if self.system_menu_window is None or not self.system_menu_window.winfo_exists():
            self.system_menu_window = System_Menu()
        else:
            self.system_menu_window.focus()
            self.system_menu_window.lift()


class ManualControl(CTkFrame):
    """Class for the manual control frame in the main window"""

    def __init__(self, parent):
        super().__init__(parent)

        self.parent = parent
        self.widgets()

    def widgets(self):
        """Function which initializes and places all the used widgets in the manual control frame"""

        pass


class Animations_Menu(CTkToplevel):
    """Content for the debug menu, aswell as generating the window 
    where the content is contained"""

    def __init__(self):
        CTkToplevel.__init__(self)
        self.geometry("400x300") # Set the dimensions of the debug window
        self.title("Animations Menu")

        # Destroy the Debug menu window, ie close the window
        def exit_button_event(): self.destroy()
        
        def command(command: str):
            msg = String()
            msg.data = command
            gui.command_publisher.publish(msg)


        self.debug_menu_label = CTkLabel(self, text="System Menu")
        self.debug_menu_label.grid(row=0, column= 0, padx= 10, pady= 5)

        self.exit_button = CTkButton(self, text="Exit Button", command= exit_button_event)
        self.exit_button.grid(row= 1, column= 0, padx= 10, pady= 5)

        self.dance_disco_button = CTkButton(self, text="Dance - Disco", command= lambda: command("/dance-disco"))
        self.dance_disco_button.grid(row= 2, column= 0, padx= 10, pady= 5)


class System_Menu(CTkToplevel):
    """Content for the debug menu, aswell as generating the window 
    where the content is contained"""

    def __init__(self):
        CTkToplevel.__init__(self)
        self.geometry("400x300") # Set the dimensions of the debug window
        self.title("System Menu")

        # Destroy the Debug menu window, ie close the window
        def exit_button_event(): self.destroy()
        
        def command(command: str):
            msg = String()
            msg.data = command
            gui.command_publisher.publish(msg)


        self.debug_menu_label = CTkLabel(self, text="System Menu")
        self.debug_menu_label.grid(row=0, column= 0, padx= 10, pady= 5)

        self.exit_button = CTkButton(self, text="Exit Button", command= exit_button_event)
        self.exit_button.grid(row= 1, column= 0, padx= 10, pady= 5)

        # Initializes the buttons for manually controlling the exo angle
        self.disconnect_button = CTkButton(self, text="Disconnect", command= lambda: command("/disconnect"))
        self.disconnect_button.grid(row= 2, column= 0, padx= 10, pady= 5)
                    
         # Initializes the buttons for manually controlling the exo angle
        self.reboot_button = CTkButton(self, text="Reboot", command= lambda: command("/reboot"))
        self.reboot_button.grid(row= 2, column= 1, padx= 10, pady= 5)

        # Initializes the buttons for manually controlling the exo angle
        self.shutdown_button = CTkButton(self, text="Shutdown", command= lambda: command("/shutdown"))
        self.shutdown_button.grid(row= 2, column= 2, padx= 10, pady= 5)



class Speech_Menu(CTkToplevel):
    def __init__(self):
        CTkToplevel.__init__(self)
        self.geometry("400x300")
        self.title("Speech Menu")

        def exit_button_event(): 
            self.destroy()

        def command(command: str):
            msg = String()
            msg.data = command
            gui.command_publisher.publish(msg)

        def clear():
            self.entry.delete(0, END)

        def submit():
            value = self.entry.get()
            command(value)
            clear()


        self.speech_menu_label = CTkLabel(self, text="Speech Menu")
        self.speech_menu_label.grid(row=0, column=0, padx=10, pady=5)

        self.exit_button = CTkButton(self, text="Exit Button", command=exit_button_event)
        self.exit_button.grid(row=1, column=0, padx=10, pady=5)

        label = CTkLabel(self, text="", font=("Helvetica", 24))
        label.grid(row=2, column=0, padx=10, pady=5)

        self.entry = CTkEntry(self,
            placeholder_text="",
            height=50,
            width=200,
            font=("Helvetica", 18),
            corner_radius=50,
            text_color="green",
            placeholder_text_color="darkblue",
            fg_color=("blue", "lightblue"),  # outer, inner
            state="normal",
        )
        self.entry.grid(row=2, column=0, padx=10, pady=5)

        submit_button = CTkButton(self, text="Submit", command=submit)
        submit_button.grid(row=3, column=0, padx=10, pady=5)

        # Bind the Enter key to the submit method
        self.entry.bind("<Return>", lambda event: submit())








####################
####    MAIN    ####
####################

# Path for 'settings.json' file
json_file_path = ".//src//pepper_wizard//pepper_wizard//settings.json"

# Instance the 'JSON_Handler' class for interacting with the 'settings.json' file
handler = JSON_Handler(json_file_path)

# Get settings from 'settings.json' file
LOG_DEBUG = handler.get_subkey_value("gui", "LOG_DEBUG")
TIMER_PERIOD = handler.get_subkey_value("gui", "TIMER_PERIOD")

# Change appearance of the GUI
set_appearance_mode('system')
set_default_color_theme("blue")

# Initialize the rclpy library
rclpy.init()

data = variables()

# Instance the node class
gui = Gui(TIMER_PERIOD, LOG_DEBUG)

while True:
    # Begin looping the node
    rclpy.spin_once(gui, timeout_sec=0.01)

    gui.app.update_idletasks()
    gui.app.update()     

    
