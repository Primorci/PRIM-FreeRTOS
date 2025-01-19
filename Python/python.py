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
publish_topic = "/STM"
subscribe_topic= "/YOLO/result"

def on_connect(client, userdata, flags, reason_code, properties):
    if reason_code.is_failure:
        print(f"Failed to connect: {reason_code}. loop_forever() will retry connection")
        errors.inc()
    else:
        print("Connected to MQTT Broker successfully.")
        client.subscribe(subscribe_topic)

def on_subscribe(client, userdata, mid, reason_code_list, properties):
    # Since we subscribed only for a single channel, reason_code_list contains
    # a single entry
    if reason_code_list[0].is_failure:
        print(f"Broker rejected you subscription: {reason_code_list[0]}")
    else:
        print(f"Broker granted the following QoS: {reason_code_list[0].value}")

def on_unsubscribe(client, userdata, mid, reason_code_list, properties):
    # Be careful, the reason_code_list is only present in MQTTv5.
    # In MQTTv3 it will always be empty
    if len(reason_code_list) == 0 or not reason_code_list[0].is_failure:
        print("unsubscribe succeeded (if SUBACK is received in MQTTv3 it success)")
    else:
        print(f"Broker replied with failure: {reason_code_list[0]}")
    client.disconnect()

# def on_message(client, userdata, msg):
#     global latest_msg
#     # Decode the message payload
#     try:
#         latest_message = json.loads(msg.payload.decode())
#         print(f"Latest MQTT Message Received: {latest_message}")
        
#         # Update Prometheus metrics (example: increment packet count)
#         packets_received.inc()
        
#         # Optionally, process the data (example: update gauges)
#         if "gyro" in latest_message:
#             gyro_x.set(latest_message["gyro"]["x"])
#             gyro_y.set(latest_message["gyro"]["y"])
#             gyro_z.set(latest_message["gyro"]["z"])
#         if "accel" in latest_message:
#             accel_x.set(latest_message["accel"]["x"])
#             accel_y.set(latest_message["accel"]["y"])
#             accel_z.set(latest_message["accel"]["z"])
#     except json.JSONDecodeError:
#         print("Error decoding MQTT message payload")
#         errors.inc()

def on_message(client, userdata, msg):
    data = json.loads(msg.payload.decode())
    if 'error' in data:
        error_message = data['error']
        if "Failed to read image" in error_message:
            print("Error detected:", error_message)
    else: 
        isDanger = data.get("detected_danger", False)  # Defaulting to False if not found
        dangerType = data.get("danger_type", [])  # Defaulting to an empty list if not found
        roadType = data.get("road_type", [])  # Defaulting to an empty list if not found

        print(f"MQTT Recived: [isDanger: {isDanger}, dangerType: {', '.join(dangerType)} roadType: {roadType}]")

        danger:bool = isDanger# True -> there is danger :: False -> there is no danger
        dangerProximity:int = random.randint(0, 100) if danger else -1 # 0 - 100 how close the danger is :: -1 no danger
        roadType:str = "A" if choose_primary_road(roadType) == "Asfalt" else "M"
        roadQuality:int = random.randint(0, 100)# 0 - 100 the quality of the road

        packet = create_packet(danger, dangerProximity, roadType, roadQuality)
        print(f"Packet: {packet}")

        parsed_data = parse_packet(packet)
        print(f"Parsed Data: {parsed_data}")

        safe_serial_write(packet)

mqttClient = mqtt.Client(client_id="STM_client", callback_api_version=mqtt.CallbackAPIVersion.VERSION2)

# Connect to MQTT broker
try:
    # Setup MQTT client
    mqttClient.on_connect = on_connect
    mqttClient.on_message = on_message
    mqttClient.on_subscribe = on_subscribe
    mqttClient.on_unsubscribe = on_unsubscribe

    mqttClient.connect(broker, port, 60)

    mqttClient.loop_start()  # Start a new thread to handle network traffic and dispatching callbacks
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
    ser = None
    errors.inc()

serial_lock = threading.Lock()

def safe_serial_read(n):
    try:
        with serial_lock:
            return ser.read(n)
    except serial.SerialException as e:
        print(f"Serial error: {e}")
        errors.inc()
    return None

def safe_serial_write(data):
    try:
        with serial_lock:
            ser.write(data)
            ser.flush()
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
    road_type_encoded = roadType.encode('utf-8')
    road_type_length = len(road_type_encoded)
    
    # Format string: header (length, type), body (data fields)
    header_format = '>HB'  # Packet length (2 bytes), Packet type (1 byte)
    body_format = f'>?iB{road_type_length}sI'  # Body format
    
    packet_length = struct.calcsize(body_format) + struct.calcsize(header_format)
    
    header = struct.pack(header_format, packet_length, 1)
    
    body = struct.pack(
        body_format,
        danger,
        dangerProximity,
        road_type_length,
        road_type_encoded,
        roadQuality
    )
    
    return header + body

road_priority = ["Asfalt", "Makedam"]

def choose_primary_road(detected_roads):
    for road_type in road_priority:
        if road_type in detected_roads:
            return road_type
    return None

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


                mqttClient.publish(publish_topic + "/gyro", json.dumps(parsed_data["gyro"]), qos=1, retain=False)
                mqttClient.publish(publish_topic + "/accel", json.dumps(parsed_data["accel"]), qos=1, retain=False)



                csvfile.flush()
        except KeyboardInterrupt:
            print("Exiting...")
        finally:
            ser.close()


if __name__ == "__main__":
    # t_sim_data = threading.Thread(target=send_yolo_data)
    t_recive_data = threading.Thread(target=recive_data)

    # t_sim_data.start()
    t_recive_data.start()
    # sim_data()


    # Join threads
    # t_sim_data.join()
    t_recive_data.join()
