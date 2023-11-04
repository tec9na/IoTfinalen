import machine
from time import sleep

class MPU6050():
    def __init__(self, i2c, addr=0x68):
        self.iic = i2c
        self.addr = addr
        try:
            self.iic.writeto(self.addr, bytearray([107, 0]))
        except:
            print("I2C failed, check that GPIO pins are connected correctly")

    def get_raw_values(self):
        # reads bytes with acceleration data from IMU memory register
        # reads from address 0x3B and next 14 bytes  
        raw_values = self.iic.readfrom_mem(self.addr, 0x3B, 14)
        return raw_values

    def bytes_toint(self, firstbyte, secondbyte):
        if not firstbyte & 0x80:
            return firstbyte << 8 | secondbyte
        return - (((firstbyte ^ 255) << 8) | (secondbyte ^ 255) + 1)

    def get_values(self):
        raw_ints = self.get_raw_values()
        vals = {}
        # updates the dictionary values to new sensor readings
        vals["acceleration x"] = self.bytes_toint(raw_ints[0], raw_ints[1]) 
        vals["acceleration y"] = self.bytes_toint(raw_ints[2], raw_ints[3])
        vals["acceleration z"] = self.bytes_toint(raw_ints[4], raw_ints[5])
        # / 340.00 + 16 converts temperature to celsius degrees
        vals["temperature celsius"] = self.bytes_toint(raw_ints[6], raw_ints[7]) / 340.00 + 16
        vals["gyroscope x"] = self.bytes_toint(raw_ints[8], raw_ints[9])
        vals["gyroscope y"] = self.bytes_toint(raw_ints[10], raw_ints[11])
        vals["gyroscope z"] = self.bytes_toint(raw_ints[12], raw_ints[13])
        return vals  # returned in range of Int16
        # -32768 to 32767

    def value_test(self):  # ONLY FOR TESTING! Also, fast reading sometimes crashes IIC      
        while 1:
            print(self.get_values())
            sleep(0.05)