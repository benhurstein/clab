#!/usr/bin/python3
import sys
import time
import paho.mqtt.client as mqtt

mqtt_host = '192.168.0.4'
pub_secs = 10

# class sensor
# topic, data (b1), counter (nb1), timestamp (last_b1_date)

class Sensor:
    def __str__(self):
        return str((self.name, self.counter, self.last_on, self.damper))

    def __init__(self, name, actuator, time_on, incr):
        self.name = name
        self.actuator = actuator
        self.counter = 0
        self.last_on = 0.0 # ultima vez visto ligado
        self.last_off = 0.0 # ultima vez desligado
        self.next_pub = time.time()+pub_secs
        self.damper = 0
        self.time_on = time_on
        self.incr = incr


mysensors = [
    Sensor('/clab/sensor/aquecedor-fluxo-b1-quente', '/clab/actuator/borboleta-B1', 10*60, 35),
    Sensor('/clab/sensor/aquecedor-fluxo-b2-quente', '/clab/actuator/borboleta-B2', 10*60, 34)
]


def get_sensordict(sensors):
    sensordict = {}
    for s in sensors:
        sensordict[s.name] = s
    return sensordict


# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))

    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    for s in mysensors:
        client.subscribe(s.name, 0)
    #client.subscribe("/clab/sensor/estar-temp-alto",0)


# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):

    global wanted_exaust
    global exaust

    sensor = userdata[msg.topic]
    data = float(msg.payload)
    now = time.time()
    if data > 0.0: #hot water on
        sensor.counter += 1
        if sensor.counter > 3:
            sensor.last_on = now
    else: # hot water off
        sensor.counter = 0

    #wanted_exaust = 0
    if now-sensor.last_on < sensor.time_on:
        if sensor.damper == 0:
            wanted_exaust += sensor.incr
        sensor.damper = 100
    else:
        if sensor.damper > 0:
            wanted_exaust -= sensor.incr
        sensor.damper = 0

    print(sensor)

    if wanted_exaust != exaust or now >= sensor.next_pub:
        print('Publish', sensor.actuator, str(sensor.damper))
        client.publish(sensor.actuator, str(sensor.damper), retain=True)
        exaust = wanted_exaust
        print('Publish', str(exaust))
        client.publish('/clab/actuator/ventilador-exaustao', str(exaust), retain=True)
        sensor.next_pub = now + pub_secs

  # if [ $wanted_exaust != $exaust -o $now -ge $proximo_envio ]; then
  #   mosquitto_pub -h $MQTT_HOST -t /clab/actuator/borboleta-B1 -m $b1_damper
  #   mosquitto_pub -h $MQTT_HOST -t /clab/actuator/borboleta-B2 -m $b2_damper
  #   exaust=$wanted_exaust
  #   mosquitto_pub -h $MQTT_HOST -t /clab/actuator/ventilador-exaustao -m $exaust
  #   proximo_envio=$[now+10]
  # fi

def main(argv):

    global wanted_exaust
    global exaust

    sensordict = get_sensordict(mysensors)

    wanted_exaust = 0
    exaust = 0

    client = mqtt.Client(userdata=sensordict)
    client.on_connect = on_connect
    client.on_message = on_message

    client.connect(mqtt_host, 1883, 60)
    # Blocking call that processes network traffic, dispatches callbacks and
    # handles reconnecting.
    # Other loop*() functions are available that give a threaded interface and a
    # manual interface.
    client.loop_forever()

if __name__ == "__main__":
    main(sys.argv)
