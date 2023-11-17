from gps_bare_minimum import GPS_Minimum       # Import af GPS_Minimun klassen fra gps_bare_minimum modulet
from machine import Pin, UART, PWM, I2C, ADC   # Import af forskellige klasser fra machine modulet 
from time import sleep, ticks_ms               # Importere funktioner fra time modulet
import umqtt_robust2 as mqtt                   # Import af umqtt_robust2 modulet som mqtt
from neopixel import NeoPixel                  # Import fra np modulet 
from mpu6050 import MPU6050                    # Import fra mpu6050 modulet som henter IMU klassen
import math                                    # Import af math modul til udregning

########################################################################

n = 12                                    # NeoPixelens 12 LED'er
np = NeoPixel(Pin(13, Pin.OUT),n)         # NP forbindes til ben 13 på ESP'eren


i2c = I2C(scl=Pin(22),sda=Pin(21))        # Her forbindes i2c til ben 22 (SCL) & 21 (SDA)
imu = MPU6050(i2c)                        # Her forbindes IMU'en til i2c objektet

buzzer = PWM(Pin(26), freq=420, duty=0)   # Buzzer forbindes til ben 26 og frekvens og duty cycle sættes


#########################################################################
# CONFIGURATION
gps_port = 2                              # ESP32 UART port, Educaboard ESP32 default UART port
gps_speed = 9600                          # UART speed, defauls u-blox speed, baud rate
#########################################################################
# OBJECTS
uart = UART(gps_port, gps_speed)          # UART object creation
gps = GPS_Minimum(uart)                   # GPS object creation
#########################################################################
bat_adc = ADC(Pin(35))                    # The battery status ADC object
bat_adc.atten(ADC.ATTN_11DB)              # Full range: 3,3 V - sltter attebtuation til 11DB så vi kan måle op til 3.3V
bat_scaling = 3.3 / 4095                  # Vores forsøg på udregning af batteriprocent
#########################################################################

# Batteridata #

def read_battery_voltage():               # Her opretter vi en funktion der læser batteriets spændning
    adc_val = bat_adc.read()              # Læser ADC værdier fra batteri ADC objektet
    voltage = bat_scaling * adc_val       # Her regnes spændningen ud fra bat_scalling og adc_val
    return voltage                        # ^ og her returneres spændningsværdien


# GPS data #


def get_adafruit_gps():                               # Her definere vi en funktion der skal hente GPS-data og sende det til adafruit 
    speed = lat = lon = None                          # Opretter variabler med None som værdi
    if gps.receive_nmea_data():                       # Denne if betingelse køres hvis der modtages korrekt GPS-data
        
        if gps.get_speed() != -999 and gps.get_latitude() != -999.0 and gps.get_longitude() != -999.0 and gps.get_validity() == "A":  # Hvis der modtages bruggbare værdier, anvendes funktionerne 
            
            # I de nææte 3 linjer af koden, gemmes returværdier fra metodekald i variabler
            speed =str(gps.get_speed()) 
            lat = str(gps.get_latitude()) 
            lon = str(gps.get_longitude())
            
            return speed + "," + lat + "," + lon + "," + "0.0"                                                    # Returnerer data med adafruit gps format
        else:                                                                                                     # else betingelsen udføres hvis ikke der modtages data fra GPS, printer nedenstående og returnere False
            print(f"GPS data to adafruit not valid:\nspeed: {speed}\nlatitude: {lat}\nlongtitude: {lon}")
            return False
    else:
        return False

# Batteri data på NeoPixel #

def bat_perc(volt):
                batt_per = 100 * ((volt - 1.65) / (2.4 - 1.65))
                return batt_per

def np_batteri(np_status_bat):                                                      # 
    if np_status_bat > 30.0 and np_status_bat <= 100.0:                             # Hvis batteriprocenten er over 30%, lyser NP LED 11 og 10 grøn
        np[11] = (0,10,0)
        np[10] = (0,10,0)
        np.write()
        
    if np_status_bat <= 30.0 and np_status_bat > 10.0:                              # Hvis batteriprocenten er under 30% og over 10 %, lyser NP LED 11 og 10 orange
        np[11] = (15,5,0)
        np[10] = (15,5,0)
        np.write()
    
    if np_status_bat <= 10.0:                                                       # Hvis batteriprocenten er under eller lig med 10%, lyser NP LED 11 og 10 rød
        np[11] = (10,0,0)
        np[10] = (10,0,0)
        np.write()


# Tacklings indikator # 
    
tackling_indikator = 0                                             # Her defineres vores variabel til registrering af antallet af tacklinger 

def set_color(tackling_indikator):                                 # Oprettelse af funktion, med inputvariablen (tackling_indikator), der kontrollere farven på LED'erne på NP og kaldes senere i koden
    np[tackling_indikator] = (10,0,10)                             # Farven er lilla, da vi kombinere rød og blå (r, g, b)
    np.write()                                                     # np.write() funktionen bruges til at "igangsætte" ændringer af farven på NP'ens LED'er

    
status = True                                                      # Hører til IMU - indikere skiftet mellem tackling og oprejst, så den ikke tæller konstant når brugeren ligger ned. Variablen status får værdien True = bruges til vores if-statement (linje 130, 133, 140, 141)

def clear_neo():                                                   # Her definere vi en funktion til at clear NP'ens LED'er
    for i in range(n):                                             # Dette er en en bundet iteration, hvor range er n, bliver prædifineret til at være 10 i en variabel. Range er en python funktion som sætter en range 
        np[i] = (0,0,0)                                            # Nulstiller NP
        np.write()                                                 # Denne funktion skubber nye tilføjelser ud (i dette tilfælde, )
        
clear_neo()                                                        # Denne funktion, som koden også indikere, rydder NP'ens LED'er



# Værdier der fastsættes inden vores while True løkke begynder 

inaktivitet_periode_ms = 30000                                     # Her sætter vi inaktivitets_periode_ms til 30 sekunder, det står som 30000 da vi bruger ticks_ms

started = False                                                    # Variablen started får værdien False, den støtter den senere betingelse (inaktivitets indikator i linje 178, 181, 191), og indikere også stopurets start
inaktivitet_starttid_ms = ticks_ms()                               # Starttids variabel for inaktivitets indikatoren, får værdien ticks_ms() = bliver kaldt og returnere en værdi i antal millisekunder (30000 ms), der udløser en "måling/tælling" af tiden efter sidste bevægelse


# Tacklings indikator #


while True:                                                        # Her starter den uendelige while True løkke
    try:                                                           # Gør brug af disse, medmindre koden skal afbrydes - try og execpt 
        
# IMU koden #

        sleep(0.01)                                                # Her benytter vi sleep funktion, med en pause på 0.01 så vores kode ikke "opheder"
        vals = imu.get_values()                                    # Her hentes der data fra IMU'ens dict (IMU'ens sensor accelerometer)
        value_acceleration_y = vals["acceleration y"]              # Vi indhenter vores key "acceleration y" fra IMU'ens dict
        print(f"ACCELERATION ER: {value_acceleration_y}")          # Vi bruger print funktionen for at få fremvist variablen, så den kan blive implementeret


        #ligge ned kode
        if vals["acceleration y"] > -5000 and status == False:     # Her oprettes 1 ud af 10 if-statements i while True løkken. Hvis begge betingelser opnås, eksekveres kodeblokken = hvis accelerationen på y-aksen overstiger -5000 og hvis variablen status er sat til False = "TACKLET"
            print("")                                              # Print funktion - dette print, printer plads/rum i vores shell mellem vores outputs
            print("spiller er: TACKLET")                           # Print funktion
            status = True                                          # Status variablen ændres i if-betingelsen, 
            if tackling_indikator < 10 :                           # Nr.2 if-statement, kommer i forlængelse af første if, som tjekker om tackling_indikator er mindre end 10 (tackling_indikator, i linje 97)
                
                set_color(tackling_indikator)                      # Funktion med kaldenavn (set_color, fra linje 99) - dets argument (tacklings_indikator, fra linje 100 )  der tænder for én neopixel, dog kun hvis variablen er under 10. Når vi kalder på set_color() overføres argumentet til funktion
                tackling_indikator = tackling_indikator+1          # Her opdateres variablen (fra linje 97) og inkrimenteres med 1 (dette betyder at den tilføjer +1 hver gang )
        
        print("")
        #stå op kode:
        if vals["acceleration y"] < -8000 and status == True:      # Dette if-statement forventer også 2xTrue for at eksevere kodeblokken = hvis accelerationen på y-aksen falder under -8000 og hvis variablen status er sat til True = "OPREJST" 
            status = False                                         # 
            print("spiller er: OPREJST")
        print("")
        
        
        
# GPS kode #
        
        gps_data = get_adafruit_gps()
        if gps_data: # hvis der er korrekt data så send til adafruit
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

        print("Stopur Igangsættes:", inaktivitet_starttid_ms)                   # Print funktion, hvor der udover en string også printes vores variabel (inaktivitet_starttid_ms) med værdien (ticks_ms()). Dette oprettes i linje 120 
        print("Tid gået:", ticks_ms() - inaktivitet_starttid_ms)                # Her trækker vi inaktivitet_starttid_ms fra ticks_ms(), i vores print(). Vores outcome er stopuret
        if gps.receive_nmea_data():                                             # if-statement der køres hvis der modtages korrekt GPS-data 
            speed = gps.get_speed()                                             # Her angiver vi variablen (speed) = gps.get_speed >< gps er en instans fra klassen GPS_Minimum, vi indhenter get_speed derfra
            print(speed)                                                        # Print, speed, da det er vores prædefineret variable
            if speed < 0.3:                                                     # tredje if betingelse, der eksekveres hvis speed er mindre end 3 (hastigheden), der efterfølges af næste if, som udføres hvis started = False (linje 119)
                if started == False:
                    inaktivitet_starttid_ms = ticks_ms()                        # Hvis started = False, ændres/opdateres "stopuret" 
                    print("genstarter tid1")                                    # Print funktion
                    started = True                                              # 
                
                if ticks_ms() - inaktivitet_starttid_ms > inaktivitet_periode_ms:   # if-statement der eksekveres hvis stopuret rammer 30 sek (30000ms) = inaktivitet_starttid_ms er større end inaktivitet_periode_ms
                    print("starter buzzer")
                    inaktivitet_starttid_ms = ticks_ms()                            # variabel (inaktivitet_starttid...) der returnere ticks_ms() som er beskrevet i linje 120
                    buzzer.duty(5)                                                  # Vi initalisere den passive buzzeres duty cycle til 5, da vi tidligere har erfaret at 10 er for højt
                    print("genstarter tid2")
                    
            if speed > 0.3:                                                         # if-statement = hvis speed er større end 0.3, afbryd buzzer og i forlængelse af det tidligere if-statement med værdien fra ticks_ms, genstart tid hvis hastighed overstiger 0.3
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
            
        if len(mqtt.besked) != 0: # Her nulstilles indkommende beskeder
            mqtt.besked = ""
        mqtt.sync_with_adafruitIO() # igangsæt at sende og modtage data med Adafruit IO
        
#################################################################################
        
    except KeyboardInterrupt:
        print("Ctrl+C pressed - exiting program.")
        sys.exit()



        
