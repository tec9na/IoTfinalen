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

buzzer = PWM(Pin(26), freq=420, duty=0)


#########################################################################
# CONFIGURATION
gps_port = 2 
gps_speed = 9600 
#########################################################################
# OBJECTS
uart = UART(gps_port, gps_speed)
gps = GPS_Minimum(uart)
#########################################################################
bat_adc = ADC(Pin(35))
bat_adc.atten(ADC.ATTN_11DB)
bat_scaling = 3.3 / 4095
#########################################################################

# Batteridata #

def read_battery_voltage():
    adc_val = bat_adc.read()
    voltage = bat_scaling * adc_val
    return voltage


# GPS data #


def get_adafruit_gps(): 
    speed = lat = lon = None 
    if gps.receive_nmea_data(): 
        
        if gps.get_speed() != -999 and gps.get_latitude() != -999.0 and gps.get_longitude() != -999.0 and gps.get_validity() == "A":
            
            
            speed = str(gps.get_speed()) 
            lat = str(gps.get_latitude()) 
            lon = str(gps.get_longitude())
            
            return speed + "," + lat + "," + lon + "," + "0.0"  
        else:
            print(f"GPS data to adafruit not valid:\nspeed: {speed}\nlatitude: {lat}\nlongtitude: {lon}")
            return False
    else:
        return False

# Batteri data på NeoPixel #

def bat_perc(volt): 
                min_spaending = 1.65
                max_spaending = 2.4
                batt_per = 100 * ((volt - min_spaending) / (max_spaending - min_spaending))
                return batt_per

def np_batteri(np_status_bat):
    if np_status_bat > 30.0 and np_status_bat <= 100.0:
        np[11] = (0,10,0)
        np[10] = (0,10,0)
        np.write()
        
    if np_status_bat <= 30.0 and np_status_bat > 10.0:  
        np[11] = (15,5,0)
        np[10] = (15,5,0)
        np.write()
    
    if np_status_bat <= 10.0: 
        np[11] = (10,0,0)
        np[10] = (10,0,0)
        np.write()


# Tacklings indikator # 
    
tackling_indikator = 0   

def set_color(tackling_indikator): 
    np[tackling_indikator] = (10,0,10) 
    np.write()

    
status = True 

def clear_neo(): 
    for i in range(n):  
        np[i] = (0,0,0) 
        np.write()
        
clear_neo()




# Værdier der fastsættes inden vores while True løkke begynder 

inaktivitet_periode_ms = 30000

started = False 
inaktivitet_starttid_ms = ticks_ms()



while True: 
    try:
        
# IMU koden #

        sleep(0.01) 
        vals = imu.get_values() 
        value_acceleration_y = vals["acceleration y"] 
        print(f"ACCELERATION ER: {value_acceleration_y}") 


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
        
        
        
# GPS kode + Batteriprocent #
        
        gps_data = get_adafruit_gps() 
        if gps_data: 
            print(f'\ngps_data er: {gps_data}') 
            
        volt = read_battery_voltage() 
        battery_percentage = bat_perc(volt) 
        
        mqtt.web_print(battery_percentage, 'tec9na/feeds/ESP32Feed') 
        sleep(3)
        
        mqtt.web_print(gps_data, 'tec9na/feeds/mapfeed/csv') 
        sleep(3)
        
        volt = read_battery_voltage() 
        np_status_bat = bat_perc(volt) 
        np_batteri(np_status_bat) 
        
        sleep(3) 
        
        
        
##############################################################################


# selvvalgt krav - inaktivitets indikator #

        print("Stopur Igangsættes:", inaktivitet_starttid_ms)  
        print("Tid gået:", ticks_ms() - inaktivitet_starttid_ms) 
        if gps.receive_nmea_data():  
            speed = gps.get_speed()
            print(speed)
            if speed < 0.3: 
                if started == False:
                    inaktivitet_starttid_ms = ticks_ms()
                    print("genstarter tid1")
                    started = True
                
                if ticks_ms() - inaktivitet_starttid_ms > inaktivitet_periode_ms:
                    print("starter buzzer")
                    inaktivitet_starttid_ms = ticks_ms() 
                    buzzer.duty(5) 
                    print("genstarter tid2")
                    
            if speed > 0.3:
                buzzer.duty(0)
                started = False
                print("stopper buzzer")
            sleep(1)

# Batteridata #
            
            print("Batterspænding er:", read_battery_voltage())
            volt = read_battery_voltage() 
            
            
            bat_perc(volt) 
            print(bat_perc(volt)) 
            
            sleep(1)
            
            
# Her sender vi og modtager data mellem ESP og Adafruit #
            
        if len(mqtt.besked) != 0:  
            mqtt.besked = ""
        mqtt.sync_with_adafruitIO()
        
#################################################################################
        
    except KeyboardInterrupt:
        print("Ctrl+C pressed - exiting program.")
        sys.exit()



        