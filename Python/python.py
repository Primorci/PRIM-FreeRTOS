import serial
import csv
import time
import struct
import random
import threading
import json

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
    producer.loop_start()  # Start a new thread to handle network traffic and dispatching callbacks
except mqtt.MQTTException as e:
    print(f"MQTT error: {e}");
    errors.inc()

PORT = "COM7"
TIMEOUT = 1

try:
    ser = serial.Serial(port=PORT, timeout=TIMEOUT)
    print(f"Serial connected to port {PORT}")
except serial.SerialException as e:
    print(f"Serial error: {e}")
    errors.inc()

serial_lock = threading.Lock()

def safe_serial_read(n):
    with serial_lock:
        return ser.read(n)

def safe_serial_write(data):
    with serial_lock:
        ser.write(data)
        ser.flush()

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
    try:
        while True:
            danger:bool = random.choice([True, False])# True -> there is danger :: False -> there is no danger
            dangerProximity:int = random.randint(0, 100) if danger else -1 # 0 - 100 how close the danger is :: -1 no danger
            roadType:str = random.choice(["A", "G"])# Asphalt or OffRoad
            roadQuality:int = random.randint(0, 100)# 0 - 100 the quality of the road

            packet = create_packet(danger, dangerProximity, roadType, roadQuality)
            print(f"Packet: {packet}")

            parsed_data = parse_packet(packet)
            print(f"Parsed Data: {parsed_data}")

            safe_serial_write(packet)

            time.sleep(1)
    except KeyboardInterrupt:
        print("Exiting...")

PACKET_FORMAT = "B H I h h h h h h"  # Header + gyroscope + accelerometer
PACKET_SIZE = struct.calcsize(PACKET_FORMAT)  # Calculate the total size

def parse_receved_packet(data):
    if len(data) != PACKET_SIZE:
        raise ValueError("Invalid packet size")

    # Unpack the binary data
    unpacked_data = struct.unpack(PACKET_FORMAT, data)

    # Map to meaningful fields
    packet = {
        "packetID": unpacked_data[0],
        "dataSize": unpacked_data[1],
        "timestamp": unpacked_data[2],
        "gyro": {
            "x": unpacked_data[3],
            "y": unpacked_data[4],
            "z": unpacked_data[5],
        },
        "accel": {
            "x": unpacked_data[6],
            "y": unpacked_data[7],
            "z": unpacked_data[8],
        }
    }

    return packet

def read_packet(serial_port, packet_size):
    """
    Reads a packet of the specified size from the serial port.
    """
    # Read the specified number of bytes
    data = serial_port.read(packet_size)
    if len(data) != packet_size:
        raise ValueError("Incomplete packet received")
    return data


def recive_data():
    # Open a CSV file to save the data
    with open("sensor_data.csv", "w", newline="") as csvfile:
        # Initialize the CSV writer
        csvwriter = csv.writer(csvfile, delimiter=';')
        # Write the header row
        csvwriter.writerow(["packetID", "dataSize", "timestamp", "gyro_x", "gyro_y", "gyro_z", "accel_x", "accel_y", "accel_z"])

        try:
            while True:
                # Read a packet from the serial port
                binary_packet = read_packet(ser, PACKET_SIZE)
                
                # Parse the packet
                parsed_data = parse_receved_packet(binary_packet)

                packets_received.inc()
                # Print the parsed data
                print("Parsed Packet Data:", parsed_data)
                
                csvwriter.writerow([
                    parsed_data["packetID"], 
                    parsed_data["dataSize"], 
                    parsed_data["timestamp"], 
                    parsed_data["gyro"]["x"], 
                    parsed_data["gyro"]["y"], 
                    parsed_data["gyro"]["z"],
                    parsed_data["accel"]["x"], 
                    parsed_data["accel"]["y"], 
                    parsed_data["accel"]["z"]
                ])
                gyro_x.set(parsed_data["gyro"]["x"])
                gyro_y.set(parsed_data["gyro"]["y"])
                gyro_z.set(parsed_data["gyro"]["z"])

                accel_x.set(parsed_data["accel"]["x"])
                accel_y.set(parsed_data["accel"]["y"])
                accel_z.set(parsed_data["accel"]["z"])


                producer.publish(topic + "/gyro", json.dumps(parsed_data["gyro"]), qos=1, retain=False)
                producer.publish(topic + "/accel", json.dumps(parsed_data["accel"]), qos=1, retain=False)



                csvfile.flush()
        except KeyboardInterrupt:
            print("Exiting...")
        finally:
            ser.close()


if __name__ == "__main__":
    t_sim_data = threading.Thread(target=sim_data)
    t_recive_data = threading.Thread(target=recive_data)

    t_sim_data.start()
    t_recive_data.start()
    # sim_data()