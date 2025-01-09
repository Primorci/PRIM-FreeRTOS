import serial
import csv
import time
import struct
import random
import threading

# MQTT lib
import time
import psutil
import paho.mqtt.client as mqtt
from prometheus_client import start_http_server, Counter, Summary, Gauge

# Prometheus metrics
packets_received = Counter('sensor_packets_received_total', 'Total number of packets received')
errors = Counter('sensor_processing_errors_total', 'Total number of errors encountered')
accel_x = Gauge('sensor_accelerometer_x', 'Accelerometer X-axis')
accel_y = Gauge('sensor_accelerometer_y', 'Accelerometer Y-axis')
accel_z = Gauge('sensor_accelerometer_z', 'Accelerometer Z-axis')
gyro_x = Gauge('sensor_gyroscope_x', 'Gyroscope X-axis')
gyro_y = Gauge('sensor_gyroscope_y', 'Gyroscope Y-axis')
gyro_z = Gauge('sensor_gyroscope_z', 'Gyroscope Z-axis')

# Start Prometheus metrics server
start_http_server(5555)

# IP address and port of MQTT Broker (Mosquitto MQTT)
broker = "10.8.1.6"
port = 1883
topic = "/STM"

def on_connect(client, userdata, flags, reasonCode, properties=None):
    if reasonCode == 0:
        print("Connected to MQTT Broker successfully.")
    else:
        print(f"Failed to connect to MQTT Broker. Reason: {reasonCode}")
        errors.inc()

def on_disconnect(client, userdata, rc):
    print(f"Disconnected from MQTT Broker. Reason: {rc}")

producer = mqtt.Client(client_id="producer_1", callback_api_version=mqtt.CallbackAPIVersion.VERSION2)

# Connect to MQTT broker
try:
    # Setup MQTT client
    producer.on_connect = on_connect
    producer.on_disconnect = on_disconnect

    producer.connect(broker, port, 60)
    # producer.loop_start()  # Start a new thread to handle network traffic and dispatching callbacks
except:
    print(f"Error: Can not connect to MQTT on port {port}, addr {broker}");
    errors.inc()

PORT = "COM7"
TIMEOUT = 1

try:
    ser = serial.Serial(port=PORT, timeout=TIMEOUT)
    print(f"Serial connected to port {PORT}")
except serial.SerialException as e:
    print(f"Serial error: {e}")
    errors.inc()


def send_command(command):
    print(command)
    command_bytes = bytes.fromhex(command)
    ser.write(command_bytes)
    ser.flush()
    print(f"Sent: {command}")

def receive_response():
    if ser.in_waiting > 0:
        response = ser.read_until().decode('utf-8')
        print(f"Received: {response.strip()}")
        return response.strip()
    return None

def create_packet(danger, dangerProximity, roadType, roadQuality):
    # Encode roadType as a length-prefixed UTF-8 string
    road_type_encoded = roadType.encode('utf-8')
    road_type_length = len(road_type_encoded)
    
    # Format string: header (length, type), body (data fields)
    header_format = '>HB'  # Packet length (2 bytes), Packet type (1 byte)
    body_format = f'>?iB{road_type_length}sI'  # Body format
    
    # Calculate packet length
    packet_length = struct.calcsize(body_format) + struct.calcsize(header_format)
    
    # Pack the header
    header = struct.pack(header_format, packet_length, 1)
    
    # Pack the body
    body = struct.pack(
        body_format,
        danger,
        dangerProximity,
        road_type_length,  # Length of the road type string
        road_type_encoded,
        roadQuality
    )
    
    # Combine header and body
    return header + body

# Helper function to decode the packet
def parse_packet(packet):
    # Unpack the header
    header_format = '>HB'
    header_size = struct.calcsize(header_format)
    packet_length, packet_type = struct.unpack(header_format, packet[:header_size])
    
    # Determine body format dynamically
    road_type_length = packet[header_size + 5]  # Length of the string is in the packet
    body_format = f'>?iB{road_type_length}sI'
    
    # Unpack the body
    body = struct.unpack(
        body_format,
        packet[header_size:]
    )
    
    # Decode roadType
    danger, dangerProximity, _, roadType, roadQuality = body
    roadType = roadType.decode('utf-8')
    
    return {
        "packet_length": packet_length,
        "packet_type": packet_type,
        "danger": danger,
        "dangerProximity": dangerProximity,
        "roadType": roadType,
        "roadQuality": roadQuality
    }

def sim_data():
    while True:
        danger:bool = random.choice([True, False])# True -> there is danger :: False -> there is no danger
        dangerProximity:int = random.randint(0, 100) if danger else -1 # 0 - 100 how close the danger is :: -1 no danger
        roadType:str = random.choice(["A", "G"])# Asphalt or OffRoad
        roadQuality:int = random.randint(0, 100)# 0 - 100 the quality of the road

        packet = create_packet(danger, dangerProximity, roadType, roadQuality)
        print(f"Packet: {packet}")

        parsed_data = parse_packet(packet)
        print(f"Parsed Data: {parsed_data}")

        try:
            ser.write(packet)
        except serial.SerialException as e:
            print(f"Error: {e}")
        print("out")
        # ser.flush()

        # time.sleep(1)

def recive_data():
    # Open a CSV file to save the data
    with open("sensor_data.csv", "w", newline="") as csvfile:
        # Initialize the CSV writer
        csvwriter = csv.writer(csvfile, delimiter=';')
        # Write the header row
        csvwriter.writerow(["Package", "Milliseconds", "X_Accelerometer", "Y_Accelerometer", "Z_Accelerometer", "X_Gyroscope", "Y_Gyroscope", "Z_Gyroscope"])

        while True:
            cur = ser.read(1)
            if cur == b'\xaa':  # Start of a valid data package
                byte = ser.read(1)
                if byte == b'\xab':
                    packets_received.inc()

                    cur = ser.read(18)
                    if len(cur) == 18:
                        # Extract accelerometer and gyroscope data
                        accel_data = struct.unpack('<hhh', cur[2:8])  # Little-endian, 3 short ints (X, Y, Z)
                        gyro_data = struct.unpack('<hhh', cur[8:14])  # Same for gyroscope

                        # Print the values (or process them)
                        print(f"Accelerometer (X, Y, Z): {accel_data}")
                        print(f"Gyroscope (X, Y, Z): {gyro_data}")
                        # Write the data to the CSV file
                        csvwriter.writerow([0, 0, accel_data[0], accel_data[1], accel_data[2], gyro_data[0], gyro_data[1], gyro_data[2]])
                        accel_x.set(accel_data[0])
                        accel_y.set(accel_data[1])
                        accel_z.set(accel_data[2])
                        gyro_x.set(gyro_data[0])
                        gyro_y.set(gyro_data[1])
                        gyro_z.set(gyro_data[2])
                        
                        # Optional: flush to ensure the data is saved periodically
                        csvfile.flush()

    ser.close()


if __name__ == "__main__":
    # t1 = threading.Thread(target=sim_data)
    # t2 = threading.Thread(target=recive_data)

    # t1.start()
    # t2.start()
    # sim_data()
    recive_data()