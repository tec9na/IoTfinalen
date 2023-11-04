import utime
import robust2
import sys
import network
import os
import _thread
from time import sleep
from time import ticks_ms
print("Forbinder til internettet...")


def sync_with_adafruitIO(): 
    # haandtere fejl i forbindelsen og hvor ofte den skal forbinde igen
    if c.is_conn_issue():
        while c.is_conn_issue():
            # hvis der forbindes returnere is_conn_issue metoden ingen fejlmeddelse
            c.reconnect()
        else:
            c.resubscribe()
    c.check_msg() # needed when publish(qos=1), ping(), subscribe()
    c.send_queue()  # needed when using the caching capabilities for unsent messages
    

try:
    from credentials import credentials
except ImportError:
    print("Credentials are kept in credentials.py, please add them there!")
    raise
# WiFi connection information
WIFI_SSID = credentials["ssid"]
WIFI_PASSWORD = credentials["password"]

# turn off the WiFi Access Point
ap_if = network.WLAN(network.AP_IF)
ap_if.active(False)

# connect the device to the WiFi network
wifi = network.WLAN(network.STA_IF)
if (wifi.isconnected()):
    wifi.disconnect()#Fixer WiFi OS fejl!
wifi.active(True)
wifi.connect(WIFI_SSID, WIFI_PASSWORD)

# wait until the device is connected to the WiFi network
MAX_ATTEMPTS = 20
attempt_count = 0
while not wifi.isconnected() and attempt_count < MAX_ATTEMPTS:
    attempt_count += 1
    utime.sleep(1)

if attempt_count == MAX_ATTEMPTS:
    print('Kunne ikke forbinde til WiFi')
    sys.exit()
    
besked = ""

def sub_cb(topic, msg, retained, duplicate):
    #print((topic, msg, retained, duplicate))
    m = msg.decode('utf-8')
    global besked
    besked = m.lower()
    print("\n",besked)
# create a random MQTT clientID
random_num = int.from_bytes(os.urandom(3), 'little')
mqtt_client_id = bytes('client_'+str(random_num), 'utf-8')

# connect to Adafruit IO MQTT broker using unsecure TCP (port 1883)
#
# To use a secure connection (encrypted) with TLS:
#   set MQTTClient initializer parameter to "ssl=True"
#   Caveat: a secure connection uses about 9k bytes of the heap
#         (about 1/4 of the micropython heap on the ESP8266 platform)
ADAFRUIT_IO_URL = credentials["ADAFRUIT_IO_URL"]
ADAFRUIT_USERNAME = credentials["ADAFRUIT_USERNAME"]
ADAFRUIT_IO_KEY = credentials["ADAFRUIT_IO_KEY"]
ADAFRUIT_IO_FEEDNAME = credentials["ADAFRUIT_IO_FEEDNAME"]


c = robust2.MQTTClient(client_id=mqtt_client_id,
                    server=ADAFRUIT_IO_URL,
                    user=ADAFRUIT_USERNAME,
                    password=ADAFRUIT_IO_KEY,
                    ssl=False)
# Print diagnostic messages when retries/reconnects happens
c.DEBUG = True
# Information whether we store unsent messages with the flag QoS==0 in the queue.
c.KEEP_QOS0 = False
# Option, limits the possibility of only one unique message being queued.
c.NO_QUEUE_DUPS = True
# Limit the number of unsent messages in the queue.
c.MSG_QUEUE_MAX = 2

c.set_callback(sub_cb)

mqtt_feedname = '{:s}/feeds/{:s}'.format(ADAFRUIT_USERNAME, ADAFRUIT_IO_FEEDNAME)

killThread = 0

def web_print2(text_in, feed):
    global killThread
    killThread = 1
    c.publish(topic=bytes(feed, 'utf-8'), msg=str(text_in))
    sleep(2)
    killThread = 0
    _thread.exit()
    
def web_print(text_in, feed = mqtt_feedname):
    if killThread == 0:
        #print(f"starting new thread \ntext_in: {text_in} \nfeed: {feed} \n killThread: {killThread}")
        _thread.start_new_thread(web_print2, (text_in, feed))
    else:
        print(f"Not sending: \nMessage: {text_in} \nTo feed: {feed}  \nplease wait 3 seconds or more before sending the next message.")


if not c.connect(clean_session=False):
    print("Forbinder til Adafruit IO, med klient ID: ",random_num)
    c.subscribe(mqtt_feedname)
    
# if not c.connect(clean_session=True):
#     print("Forbundet!")

