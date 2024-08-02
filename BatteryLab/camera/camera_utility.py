import cv2
import socket
import pickle
import struct

def send_image(ip, port):
    # Create a socket connection
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((ip, port))
    connection = client_socket.makefile('wb')

    try:
        # Open the camera
        cap = cv2.VideoCapture(0)

        while True:
            ret, frame = cap.read()
            if not ret:
                break
            
            # Serialize frame
            data = pickle.dumps(frame)
            size = len(data)

            # Send frame size and frame data
            client_socket.sendall(struct.pack(">L", size) + data)
            
    finally:
        cap.release()
        client_socket.close()

def receive_image(port):
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind(('0.0.0.0', port))
    server_socket.listen(10)
    print("Listening for connections...")

    conn, addr = server_socket.accept()
    data = b""
    payload_size = struct.calcsize(">L")

    try:
        while True:
            while len(data) < payload_size:
                data += conn.recv(4096)

            packed_msg_size = data[:payload_size]
            data = data[payload_size:]
            msg_size = struct.unpack(">L", packed_msg_size)[0]

            while len(data) < msg_size:
                data += conn.recv(4096)

            frame_data = data[:msg_size]
            data = data[msg_size:]

            frame = pickle.loads(frame_data)
            cv2.imshow('Frame', frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        conn.close()
        server_socket.close()
