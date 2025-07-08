import smach
import time
import threading
import paho.mqtt.client as mqtt

# Stato globale dei pulsanti
pulsante_giallo = False
pulsante_rosso = False
pulsante_verde = False

# MQTT Setup
MQTT_BROKER = "localhost"
TOPIC_PULSANTE_GIALLO = "macchina/pulsante_giallo"
TOPIC_PULSANTE_ROSSO = "macchina/pulsante_rosso"
TOPIC_PULSANTE_VERDE = "macchina/pulsante_verde"

TOPIC_LUCE_VERDE = "macchina/luce_verde"
TOPIC_LUCE_GIALLA = "macchina/luce_gialla"
TOPIC_LUCE_ROSSA = "macchina/luce_rossa"

client = mqtt.Client()

def on_message(client, userdata, msg):
    global pulsante_giallo, pulsante_rosso, pulsante_verde
    if msg.topic == TOPIC_PULSANTE_GIALLO:
        pulsante_giallo = bool(int(msg.payload.decode()))
    elif msg.topic == TOPIC_PULSANTE_ROSSO:
        pulsante_rosso = bool(int(msg.payload.decode()))
    elif msg.topic == TOPIC_PULSANTE_VERDE:
        pulsante_verde = bool(int(msg.payload.decode()))

client.on_message = on_message
client.connect(MQTT_BROKER)
client.subscribe([(TOPIC_PULSANTE_GIALLO, 0), (TOPIC_PULSANTE_ROSSO, 0), (TOPIC_PULSANTE_VERDE, 0)])
client.loop_start()

# Funzioni per controllare le luci
def set_luce(topic, stato):
    client.publish(topic, "on" if stato else "off")

def lampeggia_gialla(stop_event):
    while not stop_event.is_set():
        set_luce(TOPIC_LUCE_GIALLA, True)
        time.sleep(1)
        set_luce(TOPIC_LUCE_GIALLA, False)
        time.sleep(1)

# Stati SMACH
class Stato1_Attesa(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['giallo_premuto'])

    def execute(self, userdata):
        global pulsante_giallo
        print("Entrato in Stato 1: Attesa")

        # Imposta le luci
        set_luce(TOPIC_LUCE_VERDE, False)
        set_luce(TOPIC_LUCE_ROSSA, False)

        # Avvia lampeggio gialla
        stop_event = threading.Event()
        lampeggio_thread = threading.Thread(target=lampeggia_gialla, args=(stop_event,))
        lampeggio_thread.start()

        # Attendi pressione pulsante giallo
        while True:
            if pulsante_giallo:
                pulsante_giallo = False
                stop_event.set()
                lampeggio_thread.join()
                return 'giallo_premuto'
            time.sleep(0.1)

class Stato2_Verifica(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['rosso_lungo'])

    def execute(self, userdata):
        global pulsante_rosso
        print("Entrato in Stato 2")

        set_luce(TOPIC_LUCE_VERDE, True)
        set_luce(TOPIC_LUCE_GIALLA, False)
        set_luce(TOPIC_LUCE_ROSSA, False)

        start_time = None
        while True:
            if pulsante_rosso:
                if start_time is None:
                    start_time = time.time()
                elif time.time() - start_time > 4:
                    pulsante_rosso = False
                    return 'rosso_lungo'
            else:
                start_time = None
            time.sleep(0.1)

class Stato3_Emergenza(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['reset'])

    def execute(self, userdata):
        global pulsante_rosso, pulsante_verde
        print("Entrato in Stato 3: Emergenza")

        set_luce(TOPIC_LUCE_VERDE, False)
        set_luce(TOPIC_LUCE_GIALLA, False)
        set_luce(TOPIC_LUCE_ROSSA, True)

        while True:
            if pulsante_rosso and pulsante_verde:
                pulsante_rosso = False
                pulsante_verde = False
                return 'reset'
            time.sleep(0.1)

# MAIN SMACH
def main():
    sm = smach.StateMachine(outcomes=['fine'])

    with sm:
        smach.StateMachine.add('STATO1', Stato1_Attesa(), transitions={'giallo_premuto': 'STATO2'})
        smach.StateMachine.add('STATO2', Stato2_Verifica(), transitions={'rosso_lungo': 'STATO3'})
        smach.StateMachine.add('STATO3', Stato3_Emergenza(), transitions={'reset': 'STATO1'})

    outcome = sm.execute()

if __name__ == '__main__':
    main()
