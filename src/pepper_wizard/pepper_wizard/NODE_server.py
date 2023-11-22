from pepper_wizard.PEPLIB import JSON_Handler, TCP_client        
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Client(Node, TCP_client):
    """
    This is the client node of the pepper_wizard ROS2 network.
    Takes argument(s):
    TBD
    """

    def __init__(self, host, port, log_debug):

        # Initialising variables
        self.HOST = host
        self.PORT = port
        self.LOG_DEBUG = log_debug

        # Initialising the classes, from which this class is inheriting.
        Node.__init__(self, 'Client')

        self.command_subscriber = self.create_subscription(String, 'Command', self.disconnect_topic_callback, 10)
        self.command_subscriber  # prevent unused variable warning
                
        TCP_client.__init__(self, self.HOST, self.PORT, self.LOG_DEBUG)
    
    def disconnect_topic_callback(self, msg):
        self.send_data(msg.data)


####################
######  MAIN  ######
####################

def main():

    # Path for 'settings.json' file
    json_file_path = ".//src//pepper_wizard//pepper_wizard//settings.json"

    # Instance the 'JSON_Handler' class for interacting with the 'settings.json' file
    handler = JSON_Handler(json_file_path)

    # Get settings from 'settings.json' file and save them to their respective variables
    HOST = handler.get_subkey_value("client", "HOST")
    PORT = handler.get_subkey_value("client", "PORT")
    LOG_DEBUG = handler.get_subkey_value("client", "LOG_DEBUG")

    # Initialize the rclpy library
    rclpy.init()

    # Instance the ClientTCP class
    client = Client(HOST, PORT, LOG_DEBUG)

    # Begin looping the node
    rclpy.spin(client)

if __name__ == "__main__":
    main()
