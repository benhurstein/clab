#!/usr/bin/python3
import sys
import time
import paho.mqtt.client as mqtt

mqtt_host = 'localhost'
pub_secs = 10

def dif(v1, v2):
    if v1 > v2: return v1 - v2
    return v2 - v1

# class sensor
# topic, data (b1), counter (nb1), timestamp (last_b1_date)

class Sensor:
    def __str__(self):
        return str((self.name, self.value, time.time()))

    def __init__(self, name, value = -1):
        if name[0] != '/':
            name = '/clab/sensor/' + name
        self.name = name
        self._value = value
        self.last_change = time.time()

    def _get_value(self):
        return self._value
    def _set_value(self, value):
        if self.value == -1:
            self._value = value
        else:
            self._value = self._value * 0.9 + value * 0.1
        self.last_change = time.time()

    @property
    def value(self):
        return self._get_value()

    @value.setter
    def value(self, value):
        self._set_value(value)

class Actuator(Sensor):
    def __init__(self, client, name):
        if name[0] != '/':
            name = '/clab/actuator/' + name
        Sensor.__init__(self, name)
        self.client = client
        self.last_pub = 0

    def _set_value(self, value):
        now = time.time()
        envia = False
        if ( # envia se tiver grande mudanca no valor
            #    dif(value, self.value) > 0.15 * max(value, self._value)
            #or # se a mudanca e pequena mas faz mais de 10min que nao muda
                value != self.value and now - self.last_pub > 360
            or # se faz mais de hora que nao envia
                now - self.last_pub > 3600):
            #super(Actuator, self)._set_value(value)
            self._value = value
            self.last_change = now
            self.client.publish(self.name, str(value), retain=True)
            self.last_pub = now
            print(self)

sensordict = {}

sensor_keys = (
    'q1n-temp-alto',
    'q2-ambiente-temp',
    'q3-ambiente-temp',
    'estar-temp-alto',
    'escr-ambiente-temp',
    'externo-leste-temp',
    'externo-norte-temp'
    )

actuator_keys = (
    'borboleta-Q1-norte',
    'borboleta-Q1-sul',
    'borboleta-Q2',
    'borboleta-Q3',
    'borboleta-estar-sul',
    'borboleta-estar-norte',
    'borboleta-jantar',
    'borboleta-escritorio',
    'borboleta-recirculacao-superior',
    'borboleta-recirculacao-inferior',
    'ventilador-principal',
    'ventilador-recirculacao-superior',
    'ventilador-recirculacao-inferior',
    'ventilador-entrada',
    'bomba-cond-frio',
    )

def get_sensordict(keys):
    dict = {}
    for k in keys:
        s = Sensor(k)
        dict[s.name] = s
    return dict

def get_actuatordict(client, keys):
    dict = {}
    for k in keys:
        a = Actuator(client, k)
        dict[a.name] = a
    return dict

def temp_q1():
    s = sensordict['/clab/sensor/q1n-temp-alto']
    return s.value

def temp_q2():
    s = sensordict['/clab/sensor/q2-ambiente-temp']
    return s.value

def temp_q3():
    s = sensordict['/clab/sensor/q3-ambiente-temp']
    return s.value

def temp_qe():
    s = sensordict['/clab/sensor/escr-ambiente-temp']
    return s.value

def temp_qs():
    s = sensordict['/clab/sensor/estar-temp-alto']
    return s.value

def temp_ext():
    s1 = sensordict['/clab/sensor/externo-leste-temp']
    s2 = sensordict['/clab/sensor/externo-norte-temp']
    return s1.value * 0.4 + s2.value * 0.6

def wanted_temp_now(times_temps):
    t = time.localtime(time.time())
    now = t[3] + t[4] / 60.0 + t[5] / 60.0 / 60.0
    # add a first and last item on list to make sure there are items
    # before and after 'now'
    ti0, te0 = times_temps[0]
    tin, ten = times_temps[-1]
    times_temps = ((tin-24, ten),) + times_temps + ((ti0+24, te0),)
    for i in range(len(times_temps)):
        (time_after, temp_after) = times_temps[i]
        if time_after > now:
            time_before, temp_before = times_temps[i-1]
            interval_factor = (now - time_before) / (time_after - time_before)
            return temp_before + (temp_after - temp_before) * interval_factor

def wanted_temp_q1():
    return wanted_temp_now((
        (8.0, 27.2), 
        (8.0, 30.0), 
        (19.0, 30.0), 
        #(20.0, 30.0), 
        #(21.0, 28.0), 
        (20.0, 28.0), 
        (23.0, 26.7),
    ))

def wanted_temp_q2():
    return wanted_temp_now((
        #(8.0, 27.2), 
        (8.0, 30.0), 
        (19.0, 30.0), 
        #(20.0, 28.0), 
        #(21.5, 26.7), 
        (20, 27),
        (22, 30),
    ))

def wanted_temp_q3():
    return wanted_temp_now((
        (8.0, 25.0), 
        (8.0, 30.0), 

        #(19.0, 27.0),
        #(19.5,27),
        #(20.0, 26.5),
        (20.0, 30.0), 
        #(21.0, 28.5), 
        #(22.0, 25),
        #(22.0, 27.0), 
        #(24.0, 27.0), 
        #(24.0, 30.0), 
    ))

def wanted_temp_qe():
    return wanted_temp_now((
        (3.0, 24.0), 
        (5.0, 24.0), 
        (8.0, 26.9), 
        (19.5, 26.9), 
        #(20.0, 30.0), 
        (24.0, 30.0), 
    ))

def wanted_temp_qs():
    return wanted_temp_now((
        (3.0, 24.0), 
        (5.0, 24.0), 
        (8.0, 26.9), 
        (19.5, 26.9), 
        #(20.0, 30.0), 
        (24.0, 30.0), 
    ))


# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))

    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    for s in userdata.values() :
        client.subscribe(s.name, 0)


# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):

    sensor = userdata[msg.topic]
    value = float(msg.payload)

    sensor.value = value
    print(sensor)

def restrain(value, min, max):
    if value < min: return min
    if value > max: return max
    return value

def soma_positivos(l):
    t = 0
    for v in l:
        if (v > 0): t += v
    return t

def main(argv):

    global sensordict
    sensordict = get_sensordict(sensor_keys)

    client = mqtt.Client(userdata=sensordict)
    client.on_connect = on_connect
    client.on_message = on_message
    actuatordict = get_actuatordict(client, actuator_keys)

    client.connect(mqtt_host, 1883, 60)
    # Blocking call that processes network traffic, dispatches callbacks and
    # handles reconnecting.
    # Other loop*() functions are available that give a threaded interface and a
    # manual interface.
    #client.loop_forever()
    vent_princ = 0
    idq1 = idq2 = idq3 = idqe = idqs = 0
    last_sec10 = int((time.time() + 10) / 10)
    while True:
        client.loop()
        sec10 = int(time.time()/10)
        if sec10 > last_sec10:
            last_sec10 = sec10
            dq1 = temp_q1() - wanted_temp_q1()
            q1 = dq1*100 + idq1
            if q1 > 0 and q1 < 100: idq1 += dq1
            dq2 = temp_q2() - wanted_temp_q2()
            q2 = dq2*100 + idq2
            if q2 > 0 and q2 < 100: idq2 += dq2
            dq3 = temp_q3() - wanted_temp_q3()
            q3 = dq3*100 + idq3
            if q3 > 0 and q3 < 100: idq3 += dq3
            dqe = temp_qe() - wanted_temp_qe()
            qe = dqe*100 + idqe
            if qe > 0 and qe < 100: idqe += dqe
            dqs = temp_qs() - wanted_temp_qs()
            qs = dqs*100 + idqs
            if qs > 0 and qs < 100: idqs += dqs

            tot_quartos = soma_positivos((q1, q2, q3))
            tot = soma_positivos((tot_quartos, qe, qs))

            b1 = restrain(q1, 0, 100)
            b2 = restrain(q2, 0, 100)
            b3 = restrain(q3, 0, 100)
            be = restrain(qe, 0, 100)
            bs = restrain(qs, 0, 100)
            if b1 < 10: b1 = 0
            if b2 < 10: b2 = 0
            if b3 < 10: b3 = 0
            if be < 10: be = 0
            if bs < 10: bs = 0

            if q1+q2 > 150: b3 = be = bs = 0

            #idq1 += b1 - q1
            #idq2 += b2 - q2
            #idq3 += b3 - q3
            #idqe += be - qe
            #idqs += bs - qs

            mx = max(b1, b2, b3, be, bs)
            if mx != 0:
                b1 = int(b1 * 100 / mx)
                b2 = int(b2 * 100 / mx)
                b3 = int(b3 * 100 / mx)
                be = int(be * 100 / mx)
                bs = int(bs * 100 / mx)

            mintemp = min(temp_q1(), temp_q2(), temp_q3(), temp_qe(), temp_qs())
            extdelta = mintemp - temp_ext()

            # verifica necessidade de circular agua na serpentina
            extdeltap = extdelta
            if extdeltap < 0: extdeltap = 0
            if max(q1, q2) < 50 and extdelta > 1:
                actuatordict['/clab/actuator/bomba-cond-frio'].value = 0
            elif tot < 10 + 7*extdeltap:
                actuatordict['/clab/actuator/bomba-cond-frio'].value = 0
            elif tot > 50 + 7*extdeltap:
                actuatordict['/clab/actuator/bomba-cond-frio'].value = 1
            # else deixa como está

            if tot < 1:
                actuatordict['/clab/actuator/ventilador-principal'].value = 0
                ##actuatordict['/clab/actuator/ventilador-recirculacao-superior'].value = 0
                actuatordict['/clab/actuator/ventilador-recirculacao-inferior'].value = 0
                actuatordict['/clab/actuator/ventilador-entrada'].value = 0
            else:
                if tot >= 80:
                    vent_princ = 2
                elif tot < 20:
                    vent_princ = 0
                elif ((vent_princ == 2 and tot < 60)
                        or (vent_princ == 0 and tot > 40)):
                    vent_princ = 1
                actuatordict['/clab/actuator/ventilador-principal'].value = vent_princ
                vent_aux = int(tot - vent_princ*30) * 2
                if vent_aux <= 0: vent_aux = 0
                elif vent_aux < 30: vent_aux = 30
                elif vent_aux > 90: vent_aux = 90

                if extdelta > 0:
                    if max(b1, b2) < 50 and extdelta > 1:
                        actuatordict['/clab/actuator/ventilador-entrada'].value = 90
                        ##actuatordict['/clab/actuator/borboleta-recirculacao-superior'].value = 1
                    else:
                        actuatordict['/clab/actuator/ventilador-entrada'].value = vent_aux
                        ##actuatordict['/clab/actuator/borboleta-recirculacao-superior'].value = 0
                    actuatordict['/clab/actuator/borboleta-recirculacao-inferior'].value = 0
                    ##actuatordict['/clab/actuator/ventilador-recirculacao-superior'].value = 0
                    actuatordict['/clab/actuator/ventilador-recirculacao-inferior'].value = 0
                if extdelta < -0.5:
                    if vent_aux < 40: vent_aux = 40
                    if vent_princ == 1: vent_aux = min(50, vent_aux)
                    if vent_princ == 2: vent_aux = min(80, vent_aux)
                    actuatordict['/clab/actuator/ventilador-entrada'].value = 0
                    actuatordict['/clab/actuator/borboleta-recirculacao-inferior'].value = 1
                    actuatordict['/clab/actuator/ventilador-recirculacao-inferior'].value = vent_aux
                    ##if 0 < tot_quartos < 100:
                        ##actuatordict['/clab/actuator/borboleta-recirculacao-superior'].value = 1
                        ##actuatordict['/clab/actuator/ventilador-recirculacao-superior'].value = vent_aux
                    ##elif tot_quartos > 120:
                        ##actuatordict['/clab/actuator/borboleta-recirculacao-superior'].value = 0
                        ##actuatordict['/clab/actuator/ventilador-recirculacao-superior'].value = 0

            f = "{:s} {:6.3f} {:6.3f} {:6.1f} {:3d} {:.1f} {:6.1f}"
            print(f.format('q1', wanted_temp_q1(), temp_q1(), q1, b1, time.time(), idq1))
            print(f.format('q2', wanted_temp_q2(), temp_q2(), q2, b2, time.time(), idq2))
            print(f.format('q3', wanted_temp_q3(), temp_q3(), q3, b3, time.time(), idq3))
            print(f.format('qe', wanted_temp_qe(), temp_qe(), qe, be, time.time(), idqe))
            print(f.format('qs', wanted_temp_qs(), temp_qs(), qs, bs, time.time(), idqs), flush=True)
            actuatordict['/clab/actuator/borboleta-Q1-norte'].value = b1
            actuatordict['/clab/actuator/borboleta-Q1-sul'].value = b1
            actuatordict['/clab/actuator/borboleta-Q2'].value = b2
            actuatordict['/clab/actuator/borboleta-Q3'].value = b3
            actuatordict['/clab/actuator/borboleta-escritorio'].value = be
            actuatordict['/clab/actuator/borboleta-estar-norte'].value = bs
            actuatordict['/clab/actuator/borboleta-estar-sul'].value = bs
            actuatordict['/clab/actuator/borboleta-jantar'].value = bs


if __name__ == "__main__":
    main(sys.argv)
