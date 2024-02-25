import socket
import picar_4wd as fc
import threading
import time
import json
import subprocess
from video_feed import start_video_feed


def get_local_ip_address():
    try:
        ip_address = subprocess.check_output(["hostname", "-I"]).decode("utf-8").split()[0]
        return ip_address
    except subprocess.CalledProcessError:
        print("Error retrieving IP address.")
        return None


HOST = get_local_ip_address() or "192.168.0.14"  # IP address of your Raspberry PI
PORT = 65432           # Port to listen on (non-privileged ports are > 1023)
POWER = 10             # Power of motors

# ------------------------------- flask server for video feed ---------------------------------

# start flask app in separate thread for video feed
video_feed_thread = threading.Thread(target=start_video_feed)
video_feed_thread.start()


# ---------------------------------------- socket server ---------------------------------------
def get_metrics():
    metrics = {
        'battery': fc.power_read(),
        'speed': fc.speed_val(),
        'cpu_temp': fc.cpu_temperature()
    }
    return metrics


def handle_client(client, client_info):
    # continuously send metrics e.g. battery, speed, cpu temperature
    def send_data():
        while True:
            metrics = get_metrics()
            data_message = json.dumps(metrics)
            try:
                client.sendall(data_message.encode('utf-8'))
            except:
                break  # exit loop if error
            time.sleep(1)

    # send data continuously in separate thread
    data_thread = threading.Thread(target=send_data)
    data_thread.start()

    try:
        while True:
            data = client.recv(1024)  # receive 1024 Bytes of message in binary format
            print(data)
            if not data:
                break
            command = data.decode('utf-8').strip()
            print(f"Received command from {client_info}: {command}")
            if command == "start_forward":
                # Start moving forward
                fc.forward(POWER)
            elif command == "start_reverse":
                fc.backward(POWER)
            elif command == "start_left":
                fc.turn_left(POWER)
            elif command == "start_right":
                fc.turn_right(POWER)
            elif command == "stop_car":
                # Stop moving forward
                fc.stop()
            else:
                print("Invalid command received: ", command)
    except Exception as e:
        print(f"Error handling client {client_info}: {e}")
    finally:
        print("Closing server socket")
        client.close()
        data_thread.join()  # stop data sending thread


with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    s.listen()
    print(f"Server listening on {HOST}:{PORT}")
    fc.start_speed_thread()  # separate thread to calculate vehicle speed
    try:
        while True:
            client, client_info = s.accept()
            print(f"Connection from {client_info}")
            threading.Thread(target=handle_client, args=(client, client_info)).start()
    except KeyboardInterrupt:
        # Handle Ctrl+C for a graceful shutdown
        print("Server shutting down.")
    except Exception as e:
        # Handle specific exceptions based on your requirements
        print(f"Exception: {e}")
    finally:
        s.close()


############## unused #################
#
# with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
#     s.bind((HOST, PORT))
#     s.listen()
#     client, clientInfo = s.accept()
#     try:
#         while 1:
#             client, clientInfo = s.accept()
#             print("server recv from: ", clientInfo)
#             data = client.recv(1024)      # receive 1024 Bytes of message in binary format
#             if data != b"":
#                 # match action
#                 print(data)
#                 client.sendall(data) # Echo back to client
#     except:
#         print("Closing socket")
#         client.close()
#         s.close()