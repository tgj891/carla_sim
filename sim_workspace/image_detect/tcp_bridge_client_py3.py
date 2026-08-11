import socket
import numpy as np
import cv2

TCP_PORT = 9999
HOST = "127.0.0.1"
IMG_HEIGHT = 480
IMG_WIDTH = 640

def recv_all(sock, length):
    buffer = b""
    while len(buffer) < length:
        chunk = sock.recv(length - len(buffer))
        if not chunk:
            raise ConnectionError("Server disconnected")
        buffer += chunk
    return buffer


def connect_socket():
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((HOST, TCP_PORT))
    print("Connected to server at {}:{}".format(HOST, TCP_PORT))
    return sock


def main():
    while True:
        try:
            sock = connect_socket()
            while True:
                header_buf = recv_all(sock, 16)
                data_len = int(header_buf.strip())

                buffer = recv_all(sock, data_len)

                img = np.frombuffer(buffer, dtype=np.uint8).reshape(IMG_HEIGHT, IMG_WIDTH, 3)

                cv2.imshow("Carla RGB", img)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    cv2.destroyAllWindows()
                    sock.close()
                    return

        except ConnectionError:
            print("Connection lost, reconnecting...")
            cv2.destroyAllWindows()
            import time
            time.sleep(1)
        except Exception as e:
            print("Error:", e)
            break

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
