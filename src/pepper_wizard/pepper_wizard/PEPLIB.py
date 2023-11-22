import json
import socket
import serial
import serial.tools.list_ports
import time
import logging


class JSON_Handler:
    """
    Class for handling loading keys for each respective ROS2 node, subkeys for each setting and the value of each setting. 
    """
    
    def __init__(self, json_file_path):
        
        self.json_file_path = json_file_path
        self.json_obj = None

        try:
            self.json_obj = json.load(open(self.json_file_path, 'r'))
        except json.JSONDecodeError as error:
            print(f"DEBUG @ script 'EXOLIB.py' class 'JSON_Handler' function '__init__'; SYSTEM MESSAGE: Failed loading .json file with error: {error}")
            return None  # Return None if the input is not valid JSON


    def get_keys(self):
        """
        Function for getting all top level keys in a json file.
        """

        if isinstance(self.json_obj, dict):
            return list(self.json_obj.keys())
        else:                
            return None  # Return None if the input is not a JSON object
    

    def get_sublevel_keys(self, key):
        """
        Function for getting all sublevel keys of a specified key in a json file.
        """
            
        if isinstance(self.json_obj, dict):
            if key in self.json_obj and isinstance(self.json_obj[key], dict):
                return list(self.json_obj[key].keys())
            else:
                return None  # Sublevel key not found or is not a dictionary
        else:
            return None  # Input is not a JSON object
    

    def get_subkey_value(self, top_level_key, subkey):
        """
        Function for getting the value of a specified subkey of a key in a JSON file.
        """
        if isinstance(self.json_obj, dict):
            if top_level_key in self.json_obj and isinstance(self.json_obj[top_level_key], dict):
                if subkey in self.json_obj[top_level_key]:
                    return self.json_obj[top_level_key][subkey]
                else:
                    return None  # Subkey not found
            else:
                return None  # Top-level key not found or is not a dictionary
        else:
            return None  # Input is not a JSON object
        


class TCP_Server:
    """
    Creates a TCP server, which awaits connection from any source and then continously recieves data on the socket.
    Takes parameters:
       Host - Server host machine IP
       Port - Server host port to communicate on
       Debug - bool print statements (defaults False)
    """

    def __init__(self, Host, Port, Debug=False):
        
        # init variables
        self.HOST = Host
        self.PORT = Port
        self.DEBUG = Debug
        self.data_string = None

        # Create a socket object
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        
        # Bind the socket to a specific address and port
        self.socket.bind((self.HOST, self.PORT))

        
    def await_connection(self):
        """
        While true loop for establishing socket connections. 
        When a connection is established then recieve_data_loop function is called where data is continously recieved in a while true loop. 
        If the connection is then broken, the recieve_data_loop is broken and the flow returns here where the system awaits a new connection.
        """
        
        while True:

            # Listen for incoming connections
            self.socket.listen(1)

            # If the DEBUG flag is raised, we print data to terminal
            if self.DEBUG:
                print(f"DEBUG @ script 'EXOLIB.py' class 'serverTCP' function 'await_connection'; SYSTEM MESSAGE: Waiting for client")
            else:
                print("Waiting for client")
            
            # Wait for a client to connect (accepting any incoming connection)
            conn, addr = self.socket.accept()

            # If the DEBUG flag is raised, we print data to terminal
            if self.DEBUG:
                print(f"DEBUG @ script 'EXOLIB.py' class 'serverTCP' function 'await_connection'; SYSTEM MESSAGE: Client connected")
            else:
                print("Client connected")

            if conn:
                return conn


    def recieve_data_loop(self, connection):
        """
        While true loop handling recieving of data on the socket connection.
        If the connection is broken, then the recieved data is empty. When this happens the loop breaks.
        This function runs when a connection is established in await_connection function.
        When the loop breaks, the flow returns to await_connection.
        """

        # Receive data from the client
        data = connection.recv(1024)

        # Format the data as a string
        self.data_string = f"{str(data.decode())}"

        # If the data is empty, then the connection has been broken. Therefore no more data will arrive and the system shall attempt to reconnect.
        if self.data_string == "":
            self.await_connection()

        # If the DEBUG flag is raised, we print data to terminal
        if self.DEBUG:
            print(f"DEBUG @ script 'EXOLIB.py' class 'serverTCP' function 'recieve_data_loop'; VARIABLE 'data_string': {self.data_string}")

        return self.data_string
    

class TCP_client:
    """
    Creates a TCP client that connects to the specified host and port and sends data to the server.
    """

    def __init__(self, host, port, log_debug=True):
        # Initialize variables
        self.HOST = host
        self.PORT = port
        self.LOG_DEBUG = log_debug

        self.logger = logging.getLogger()

        # Create a socket object
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        self.connect_to_server()

    def connect_to_server(self):
        # Connect to the server
        try:
            self.socket.connect((self.HOST, self.PORT))
            self.logger.info(f"Connected to server @ HOST:PORT {self.HOST}:{self.PORT}")

        except Exception as e:
            self.logger.error(f"Failed to connect to server with error: {e}")


    def send_data(self, data: str):        
        try:
            # Send the message to the server
            self.socket.sendall(data.encode())

            if data.lower() == '/disconnect':
                self.logger.info("Disconnecting")

                time.sleep(2)

                quit()

        except Exception as e:
            self.logger.error(f"Failed to send message with error: {e}")