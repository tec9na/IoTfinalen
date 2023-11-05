from gps_bare_minimum import GPS_Minimum
from machine import Pin, UART, PWM, I2C, ADC
from time import sleep, ticks_ms
import umqtt_robust2 as mqtt
from neopixel import NeoPixel
from mpu6050 import MPU6050
import math

########################################################################

n = 12
np = NeoPixel(Pin(13, Pin.OUT),n)


i2c = I2C(scl=Pin(22),sda=Pin(21))
imu = MPU6050(i2c)

#########################################################################
# CONFIGURATION
gps_port = 2                               # ESP32 UART port, Educaboard ESP32 default UART port
gps_speed = 9600                           # UART speed, defauls u-blox speed
#########################################################################
# OBJECTS
uart = UART(gps_port, gps_speed)           # UART object creation
gps = GPS_Minimum(uart)                    # GPS object creation
#########################################################################
bat_adc = ADC(Pin(35))        # The battery status ADC object
bat_adc.atten(ADC.ATTN_11DB)           # Full range: 3,3 V
bat_scaling = 3.3 / 4095
#########################################################################
buzzer = PWM(Pin(26), freq=420, duty=0)

# Batteridata #

def read_battery_voltage():
    adc_val = bat_adc.read()
    voltage = bat_scaling * adc_val
    return voltage

read_battery_voltage()
print(bat_adc.read())
print(read_battery_voltage())

def battery_percentage():
    percentage = (read_battery_voltage() / 4.2) * 100
    return percentage

battery_percentage()
print(battery_percentage())

# GPS data


def get_adafruit_gps():
    speed = lat = lon = None # Opretter variabler med None som værdi
    if gps.receive_nmea_data():
        # hvis der er kommet end bruggbar værdi på alle der skal anvendes
        if gps.get_speed() != -999 and gps.get_latitude() != -999.0 and gps.get_longitude() != -999.0 and gps.get_validity() == "A":
            # gemmer returværdier fra metodekald i variabler
            speed =str(gps.get_speed()) 
            lat = str(gps.get_latitude()) 
            lon = str(gps.get_longitude())
            # returnerer data med  adafruit gps format
            return speed + "," + lat + "," + lon + "," + "0.0"
        else: # hvis ikke både hastighed, latitude og longtitude er korrekte 
            print(f"GPS data to adafruit not valid:\nspeed: {speed}\nlatitude: {lat}\nlongtitude: {lon}")
            return False
    else:
        return False

def np_batteri(np_status_bat):
    if np_status_bat > 30.0 and np_status_bat <= 100.0:
        np[11] = (0,10,0)
        np[10] = (0,10,0)
        print("green")
        
        np.write()
    if np_status_bat <= 30.0 and np_status_bat > 10.0:
        np[11] = (15,5,0)
        np[10] = (15,5,0)
        np.write()
        print("orange")
    
    if np_status_bat <= 10.0:
        np[11] = (10,0,0)
        np[10] = (10,0,0)
        print("red")
        np.write()
    
    
tackling_indikator = 0

def set_color(tackling_indikator):
    np[tackling_indikator] = (10,0,10)
    np.write()

    
status = True #hører til IMU

def clear_neo():
    for i in range(n):
        np[i] = (0,0,0)
        np.write()
        
clear_neo()

inaktivitet_periode_ms = 30000

started = False
inaktivitet_starttid_ms = ticks_ms()
# Tacklings indikator

while True:
    try:
        
        #IMU koden kommer her:#############################################################
        ###### printer hele dictionary som returneres fra get_values metoden, er dog udkommenteret lige nu
        #print(imu.get_values()) 
        sleep(0.01)
        vals = imu.get_values()
        value_acceleration_y = vals["acceleration y"]
        print(f"ACCELERATION ER: {value_acceleration_y}")
        #print(vals["acceleration x"])
            
        #ligge ned kode
        if vals["acceleration y"] > -5000 and status == False:
            print("")
            print("spiller er: TACKLET")
            status = True
            if tackling_indikator < 10 :
                
                set_color(tackling_indikator)
                tackling_indikator = tackling_indikator+1
        
        print("")
        #stå op kode:
        if vals["acceleration y"] < -8000 and status == True:
            status = False
            print("spiller er: OPREJST")
        print("")
        
        
        
        #GPS koden:##############################################################
        
        gps_data = get_adafruit_gps()
        if gps_data: # hvis der er korrekt data så send til adafruit
            print(f'\ngps_data er: {gps_data}')
        percentage = battery_percentage()
        mqtt.web_print(percentage, 'tec9na/feeds/ESP32Feed')
        sleep(3)
        mqtt.web_print(gps_data, 'tec9na/feeds/mapfeed/csv')
        sleep(3)
        
        #mqtt.web_print(battery_percentage())
        np_batteri(battery_percentage())
        
        sleep(3) 
        
        
        
        #Her sender vi og modtager data mellem ESP og Adafruit##############################

        if len(mqtt.besked) != 0: # Her nulstilles indkommende beskeder
            mqtt.besked = ""            
        mqtt.sync_with_adafruitIO() # igangsæt at sende og modtage data med Adafruit IO
        

##############################################################################

# selvvalgt krav - inaktivitets indikator 
        print("inaktivitet_starttid_ms", inaktivitet_starttid_ms)
        print("tid gået", ticks_ms() - inaktivitet_starttid_ms)
        if gps.receive_nmea_data():
            speed = gps.get_speed()
            print(speed)
            if speed < 0.8:
                if started == False:
                    inaktivitet_starttid_ms = ticks_ms()
                    print("genstarter tid1")
                    started = True
                
                if ticks_ms() - inaktivitet_starttid_ms > inaktivitet_periode_ms:
                    print("starter buzzer")
                    inaktivitet_starttid_ms = ticks_ms()
                    buzzer.duty(10)
                    print("genstarter tid2")
                    
            if speed > 0.8:
                buzzer.duty(0)
                started = False
                print("stopper buzzer")
            sleep(1)
        
####################################################################################
        
    except KeyboardInterrupt:
        print("Ctrl+C pressed - exiting program.")
        sys.exit()



