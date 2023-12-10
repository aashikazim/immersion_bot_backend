import socket

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def send(data, ip, port):
    sock.sendto(data.encode("utf-8"), (ip, port))


# import socket
# import threading

# class ThreadedUDPSender(threading.Thread):
#     def __init__(self, ip, port):
#         super(ThreadedUDPSender, self).__init__()
#         self.ip = ip
#         self.port = port
#         self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#         self.data = ""

#     def send(self, data):
#         if data:
#             self.sock.sendto(data.encode("utf-8"), (self.ip, self.port))


