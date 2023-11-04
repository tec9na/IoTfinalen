# A simple GPS library parsing $GPRMC
# Developed on u-blox NEO-7M, milliseconds are not parsed

class GPS_Minimum:
    __nmea_buffer = ""                       # NMEA receive buffer
    __utc_year = 0                           # UTC
    __utc_month = 0
    __utcDay = 0
    __utc_hours = 0
    __utc_minutes = 0
    __utc_seconds = 0
    __latitude = -999.0                     # Decimal degrees
    __longitude = -999.0                    # Decimal degrees
    __validity = "V"                        # Void
    __speed = 0
    __course = 0.0
    
    def __init__(self, uart, all_nmea = False):
        self.uart = uart
        self.all_nmea = all_nmea

        # Enable relevant and wanted NMEA frames
        # more infom about $GPRMC https://aprs.gids.nl/nmea/#rmc 
        uart.write("$PUBX,40,RMC,1,1,1,0*46\n") # Make sure the $GPRMC are always enabled
        # Disable all but the $GPRMC frames
        uart.write("$PUBX,40,GGA,0,0,0,0*5A\n")
        uart.write("$PUBX,40,ZDA,0,0,0,0*44\n")
        uart.write("$PUBX,40,GLL,0,0,0,0*5C\n") 
        uart.write("$PUBX,40,GRS,0,0,0,0*5D\n")
        uart.write("$PUBX,40,GSA,0,0,0,0*4E\n")
        uart.write("$PUBX,40,GST,0,0,0,0*5B\n")
        uart.write("$PUBX,40,GSV,0,0,0,0*59\n")
        uart.write("$PUBX,40,VTG,0,0,0,0*5E\n")            

    def __parse_nmea_frame(self, string):   # Change to parse all relevant frames: http://aprs.gids.nl/nmea/, no checksum validation
        
        sub_frame = string.split(',')        # Split the NMEA frame into parts
        
        if len(sub_frame[0]) < 6:            # No real data received
            return

        # Parse $GPRMC                        
        elif sub_frame[0] == "$GPRMC":       # $GPRMC,081836.00,A,3751.65,S,14507.36,E,000.0,360.0,130998,011.3,E*62
            # UTC hours, minutes and seconds
            if len(sub_frame[1]) > 5:
                self.__utc_hours = int(sub_frame[1][0:2])
                self.__utc_minutes = int(sub_frame[1][2:4])
                self.__utc_seconds = int(sub_frame[1][4:6])
        
            # Validity
            if len(sub_frame[2]) > 0:
                self.__validity = sub_frame[2]
                
            # Latitude
            if len(sub_frame[3]) > 0:
                laf = float(sub_frame[3])
                lai = int(laf / 100)
                lad = (laf - lai * 100) / 60.0  # Convert to decimal degrees
                self.__latitude = lai + lad
                if sub_frame[4] == "S":
                    self.__latitude = -latitude                

            # Longitude
            if len(sub_frame[5]) > 0:
                lof = float(sub_frame[5])
                loi = int(lof / 100)
                lod = (lof - loi * 100) / 60.0  # Convert to decimal degrees
                self.__longitude = loi + lod
                if sub_frame[6] == "W":
                    self.__longitude = -longitude
        
            # Speed, km/t
            if len(sub_frame[7]) > 0:
                self.__speed = float(sub_frame[7]) * 1.852
        
            # Course, °
            if len(sub_frame[8]) > 0:
                print(sub_frame[8])
                self.__course = float(sub_frame[8])
        
            # UTC year, month, day
            if len(sub_frame[9]) > 5:
                self.__utcDay = int(sub_frame[9][0:2])
                self.__utc_month = int(sub_frame[9][2:4])
                self.__utc_year = 2000 + int(sub_frame[9][4:6])
        
    def get_utc_year(self):
        return self.__utc_year

    def get_utc_month(self):
        return self.__utc_month

    def get_utc_day(self):
        return self.__utcDay
    
    def get_utc_hours(self):
        return self.__utc_hours

    def get_utc_minutes(self):
        return self.__utc_minutes

    def get_utc_seconds(self):
        return self.__utc_seconds

    def get_latitude(self):
        return self.__latitude

    def get_longitude(self):
        return self.__longitude
   
    def get_validity(self):
        return self.__validity
    
    def get_speed(self):
        return self.__speed   
    
    def get_course(self):
        return self.__course
    
    def get_frames_received(self):
        return self.__frames_received   
    
    def clear_frames_received(self):
        self.__frames_received = 0

    def write(self, string):
        self.uart.write(string, end = '')
        return 
    
    # The receiver funtion, call at least once per second
    def receive_nmea_data(self, echo = False):           # Returns true if data was parsed, otherwise false
        self.__nmea_buffer
        
        if self.uart.any() > 0:
            string = self.uart.readline()                # Collect incoming chars
            try:
                self.__nmea_buffer += string.decode("utf-8")  # UART returns bytes. They have to be conv. to chars/a string
            
                if "\n" in self.__nmea_buffer:
                    self.__parse_nmea_frame(self.__nmea_buffer)
                    if echo:
                        print(self.__nmea_buffer, end = '')   # Echo the received frame
                    self.__nmea_buffer = ""
              
                    return True
            except ValueError as e:
                print(f"Failed to parse NMEA sentence with error: {e}")
                return False
            except:
                print("An error hapened while parsing NMEA sentence")
                return False
            
        return False
         
# EOF ##################################################################