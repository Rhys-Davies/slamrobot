import socket
import struct
import time

# Quick and dirty networking library.
# Author: Rhys Davies


class server(object):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    def __init__(self, port = 9292, host = ''):
        self.sock.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
        self.host = host
        self.port = port


    def wait_and_connect(self):

        self.sock.bind((self.host, self.port))
        self.sock.listen(0)
        print('Server Listening: ', self.sock.getsockname())
        self.conn, self.addr = self.sock.accept()
        print('Server Accepted')


    def send_msg(self,msg):
        msg = struct.pack('>I', len(msg)) + bytes(msg, 'utf-8')
        #print('Struct Packed')
        self.conn.sendall(msg)

    def wait_and_rcv(self):
        self.sock.bind((self.host, self.port))
        self.sock.listen(0)
        print('Server Listening: ', self.sock.getsockname())
        self.conn, self.addr = self.sock.accept()
        print('Server Accepted')
        data = self.recv_msg()
        return data

    def recv_msg(self):
        # Read message length and unpack it into an integer
        raw_msglen = self.recvall(4)
        if not raw_msglen:
            return None
        msglen = struct.unpack('>I', raw_msglen)[0]
        print(str(msglen))
        # Read the message data
        return self.recvall(msglen)

    def recvall(self, n):
        # Helper function to recv n bytes or return None if EOF is hit
        data = bytes()
        while len(data) < n:
            packet = self.conn.recv(n - len(data))
            if not packet:
                #return None
                print('Not packet')
            data += packet
        return data
            
    def quit(self):
        self.sock.close()


class client(object):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    def __init__(self, host='localhost', port=9292):
        self.host = host
        self.port = port

    def begin_session(self, new_host):
        self.host = new_host
        self.sock.connect((self.host, self.port))
        #self.sock.send(bytes(proto.START_SESSION, 'utf-8'))
    
    def client_send(self,msg):
        print('Server Accepted')
        msg = struct.pack('>I', len(msg)) + bytes(msg, 'utf-8')
        #print('Struct Packed')
        self.sock.sendall(msg)

    def recv_msg(self):
        # Read message length and unpack it into an integer
        raw_msglen = self.recvall(4)
        if not raw_msglen:
            return None
        msglen = struct.unpack('>I', raw_msglen)[0]
        # Read the message data
        return self.recvall(msglen)
        self.quit()

    def recvall(self, n):
        # Helper function to recv n bytes or return None if EOF is hit
        data = bytes()
        while len(data) < n:
            packet = self.sock.recv(n - len(data))
            if not packet:
                #return None
                print('Not packet')
                time.sleep(0.2)
            data += packet
        return data
   
    def quit(self):
        self.sock.close()
