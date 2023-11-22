import socket
import time
import logging

class ClientTCP:
    """
    Creates a TCP client that connects to the specified host and port and sends data to the server.
    """

    def __init__(self, host, port, debug=True):
        # Initialize variables
        self.HOST = host
        self.PORT = port
        self.DEBUG = debug

        self.logger = logging.getLogger()

        # Create a socket object
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)


    def connect_to_server(self):
        # Connect to the server
        self.socket.connect((self.HOST, self.PORT))
        self.logger.info(f"Connected to server @ HOST:PORT {self.HOST}:{self.PORT}")

    def send_data(self):
        while True:
            message = input("")
            
            try:
                # Send the message to the server
                self.socket.sendall(message.encode())

                if message.lower() == '/disconnect':
                    self.logger.info("Disconnecting")

                    time.sleep(2)

                    break

            except Exception as e:
                if self.DEBUG:
                     self.logger.error(f"Failed to send message with error: {e}")


def main():
    ## Define server host ip
    HOST = "192.168.1.5"

    ## Define server port
    PORT = 20000

    ## Variable for enabling/disabling print statements
    DEBUG = True

    # Instance the ClientTCP class
    client = ClientTCP(HOST, PORT, DEBUG)

    # Connect to the server
    client.connect_to_server()

    # start sending data continously from user input
    client.send_data()

if __name__ == '__main__':
    main()
