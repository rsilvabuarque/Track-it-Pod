import machine
import ubinascii
import utime
import ustruct
from machine import UART
from machine import Pin
from lcd_api import LcdApi
from pcf8574 import PCF8574
from lcd import Lcd

# Initialize the UART2 port for the HC-12 module
uart_hc12 = UART(2, baudrate=9600, bits=8, parity=None, stop=1, tx=machine.Pin(4), rx=machine.Pin(5))

# Initialize the I2C bus and the PCF8574 GPIO expander
i2c = machine.I2C(0, scl=machine.Pin(22), sda=machine.Pin(21))
pcf = PCF8574(i2c, 0x27)

# Initialize the LCD display
api = LcdApi(i2c)
lcd = Lcd(api)

# Read the GPS data sent by the other ESP32C3 and print it on the LCD display
def read_gps_and_display():
    while True:
        try:
            # Read a binary message from the HC-12 module
            msg = uart_hc12.read(12)
            if msg:
                # Unpack the message into variables
                lat, lon, alt, num_sats, sog, track_angle, gps_error = ustruct.unpack("<fffffBB", msg)
                # Clear the LCD display
                lcd.clear()
                # Print the GPS data on the LCD display
                lcd.putstr("Lat: %.6f\nLon: %.6f\nAlt: %.2f\nSat: %d\nSOG: %.2f\nTrA: %.2f\nErr: %d" % (lat, lon, alt, num_sats, sog, track_angle, gps_error))
            # Wait for some time before reading the next GPS data
            utime.sleep(0.5)
        except Exception as e:
            print("Error reading GPS data:", e)
