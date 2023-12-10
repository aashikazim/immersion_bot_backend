import socket
import threading
import json
import time
from command_type import CommandType

class UDPReceiver(threading.Thread):
    def __init__(self, host, port, callback_func):
        super(UDPReceiver, self).__init__()
        self.host = host
        self.port = port
        self.callback_func = callback_func
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(None)
        self.sock.bind((self.host, self.port))

    def run(self):
        while True:
            data, addr = self.sock.recvfrom(1024)
            self.callback_func(data, addr)

# class UdpReceiver:
#     def __init__(self, ip, port):
#         self.ip = ip
#         self.port = port
#         self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#         self.sock.settimeout(None)
#         self.sock.bind((self.ip, self.port))
#         self.running = False
#         self.is_data_received = False
#         self.parsed_data =  None
#         self.prev_manual_time = None

#     def start(self):
#         self.running = True
#         self.thread = threading.Thread(target=self.receive_data)
#         self.thread.start()

#     def receive_data(self):
#         while self.running:
#             try:
#                 data, addr = self.sock.recvfrom(1024)  # buffer size is 1024 bytes
#                 print("Received Actual: ", data)
#                 if self.is_data_received == False:
#                     self.parsed_data = json.loads(data)
#                     if self.parsed_data["command"] == CommandType.MANUAL_CONTROL:
#                         self.prev_manual_time = time.time()

#                     # print("Received message: ", self.parsed_data)
#                     self.is_data_received = True

#                 # Here, you can add code to process the received data or integrate with ROS
#             except Exception as e:
#                 print("Error receiving data: ", e)

#     def stop(self):
#         """Stop the UDP receiver."""
#         self.running = False
#         self.sock.close()
#         self.thread.join()