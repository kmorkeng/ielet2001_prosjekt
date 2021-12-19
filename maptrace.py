'''
This Example sends harcoded data to Ubidots using the Paho MQTT
library.

Please install the library using pip install paho-mqtt

Made by Jose García @https://github.com/jotathebest/
'''

import paho.mqtt.client as mqttClient
import paho.mqtt.subscribe as subscribe
import time
import json
import random

'''
global variables
'''

connected = False  # Stores the connection status
BROKER_ENDPOINT = "industrial.api.ubidots.com"
PORT = 1883
MQTT_USERNAME = "BBFF-8eYG4XFUFDUWjtzgxbu7WMXwj2EavJ"
MQTT_PASSWORD = ""
TOPIC = "/v1.6/devices/"
DEVICE_LABEL = "maptrace"
VARIABLE_LABEL = "pos"
VARIABLE_LABEL_LAT = "latitudes"
VARIABLE_LABEL_LONG = "longitudes"
SUBSCRIBE_DEVICE_LABEL = "test"
SUBSCIBE_VARIABLE_LABEL = "getcoordinates"

upload = False



'''
Functions to process incoming and outgoing streaming
'''


def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("[INFO] Connected to broker")
        global connected  # Use global variable
        connected = True  # Signal connection

    else:
        print("[INFO] Error, connection failed")


def on_publish(client, userdata, result):
    print("[INFO] Published!")

def print_msg(client, userdata, message):
    payloadString = str(message.payload) #gjør om bytstreamen til in string slik at den kan indekses
    print("%s : %s" % (message.topic, message.payload))
    value = payloadString[12] #henter verdien som er en er interessert i
    if(value == "1"): #hvis verdien er lik en så skal vi unsubscribe og laste opp koordinater
        client.disconnect()
        upload = True
def on_message(client, userdata, message):
    print("message received " ,str(message.payload.decode("utf-8")))
    print("message topic=",message.topic)
    print("message qos=",message.qos)
    print("message retain flag=",message.retain)

def connect(mqtt_client, mqtt_username, mqtt_password, broker_endpoint, port):
    global connected

    if not connected:
        mqtt_client.username_pw_set(mqtt_username, password=mqtt_password)
        mqtt_client.on_connect = on_connect
        mqtt_client.on_publish = on_publish
        mqtt_client.connect(broker_endpoint, port=port)
        mqtt_client.loop_start()

        attempts = 0

        while not connected and attempts < 5:  # Waits for connection
            print("[INFO] Attempting to connect...")
            time.sleep(1)
            attempts += 1

    if not connected:
        print("[ERROR] Could not connect to broker")
        return False

    return True


def publish(mqtt_client, topic, payload):
    try:
        mqtt_client.publish(topic, payload)
    except Exception as e:
        print("[ERROR] There was an error, details: \n{}".format(e))


def main(mqtt_client,lat,lng):

    # Simulates sensor values
    #sensor_value = random.random() * 100
    sensor_value = 1
    #lat = 6.101;
    #lng= -71.293;

    # Builds Payload and topíc
    payload = {VARIABLE_LABEL: {"value": sensor_value,
                                "context": {"lat": lat, "lng": lng}}
                                }
    payloadLat = {VARIABLE_LABEL_LAT: {"value": lat,"context": {"lat": lat, "lng": lng}}}
    payloadLong = {VARIABLE_LABEL_LONG: {"value": lng,"context": {"lat": lat, "lng": lng}}}
                                
    payload = json.dumps(payload)
    payloadLat = json.dumps(payloadLat)
    payloadLong = json.dumps(payloadLong)

    topic = "{}{}".format(TOPIC, DEVICE_LABEL)
    print(topic)

    if not connected:  # Connects to the broker
        connect(mqtt_client, MQTT_USERNAME, MQTT_PASSWORD,
                BROKER_ENDPOINT, PORT)

    # Publishes values
    print("[INFO] Attempting to publish payload:")
    print(payload)
    for x in range(6):
        publish(mqtt_client, topic, payloadLong)
        time.sleep(0.5)
        publish(mqtt_client, topic, payloadLat)
        time.sleep(0.5)
        publish(mqtt_client, topic, payload)
        time.sleep(0.5)
    
    
    

latitudes = []
longitudes = []

with open('maptrace.txt') as f:
    lines = f.readlines()
    i = 0
    while i < len(lines): #formatterer og leser inn koordinatene fra tekstfilene i en liste
        if (i != 0) and (i < len(lines)-1):
            lines[i] = lines[i].strip()
            split_string = lines[i].split("\t")

            latitudes.append(split_string[1])
            longitudes.append(split_string[2])
        i+=1

print(latitudes,longitudes)


mqtt_client = mqttClient.Client()
mqtt_client.on_message=on_message
i = 0
if(upload == False):
    #mqtt_client.subscribe("/v1.6/devices/test/getcoordinates")
    time.sleep(0.1)
    subscribe.callback(print_msg, "/v1.6/devices/test/getcoordinates", hostname=BROKER_ENDPOINT,auth = {'username':MQTT_USERNAME})
    
    print("123")

upload = True

while (i < len(latitudes)):
    main(mqtt_client,latitudes[i],longitudes[i])
    time.sleep(5)
    i+=1
upload = False