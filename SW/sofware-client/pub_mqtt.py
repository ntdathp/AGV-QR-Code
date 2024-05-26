import paho.mqtt.client as mqtt
import json
import time

def on_connect(client, userdata, flags, rc):
    # print(f"Connected with result code {rc}")

    # Send a message to the raspberry/topic every 1 second, 5 times in a row
        # Constructing the message as a JSON string
    message = {
        "method": "PROCESS_LIST",
        "payload": {
            "path": "6, 6, 6, 7, 6, 6, 4|3, 1 ",
            "velocity": "normal"
        }
    }
    # Convert message to JSON format with indentation for readability
    json_message = json.dumps(message, indent=4) + 'k'
    # The four parameters are topic, sending content, QoS and whether retaining the message respectively
    client.publish('AGV/AGV_01/control', payload=json_message, qos=0, retain=False)
    print(f"{json_message}")

client = mqtt.Client()
client.on_connect = on_connect
client.connect("localhost", 1883)

# Start the loop in a separate thread
client.loop_start()

# Stop the loop and exit the program
client.loop_stop()