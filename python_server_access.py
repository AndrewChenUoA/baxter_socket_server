import socket

#Create socket and send command
def sendSocket(command):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        sock.connect(("192.168.1.114",10111))
        sock.sendall(command + "\n")
        received = sock.recv(1024)
    finally:
        sock.close()
        
    return received
