import machine
import ubinascii
import utime
import ustruct
import pynmea2
from machine import UART

# Initialize the UART1 port for the NEO-6M GPS module
uart_gps = UART(1, baudrate=9600, bits=8, parity=None, stop=1, tx=machine.Pin(2), rx=machine.Pin(3))

# Initialize the UART2 port for the HC-12 module
uart_hc12 = UART(2, baudrate=9600, bits=8, parity=None, stop=1, tx=machine.Pin(4), rx=machine.Pin(5))

# Read the GPS data from the UART1 port and send it through the HC-12 module
def read_gps_and_send():
    while True:
        try:
            # Read a line of NMEA sentence from the GPS module
            nmea = uart_gps.readline()
            if nmea.startswith(b'$GPRMC'):
                try:
                    # Parse the $GPRMC sentence using pynmea2 library
                    data = pynmea2.parse(nmea.decode('ascii'))
                    # Extract the latitude, longitude, speed over ground, and track angle
                    lat = data.latitude
                    lon = data.longitude
                    sog = data.spd_over_grnd
                    track_angle = data.true_course
                except Exception as e:
                    print("Error parsing GPRMC data:", e)
                    continue
            elif nmea.startswith(b'$GPGGA'):
                try:
                    # Parse the $GPGGA sentence using pynmea2 library
                    data = pynmea2.parse(nmea.decode('ascii'))
                    # Extract the altitude, number of satellites, and GPS error
                    alt = data.altitude
                    num_sats = data.num_sats
                    gps_error = data.gps_qual
                except Exception as e:
                    print("Error parsing GPGGA data:", e)
                    continue
                # Pack the extracted data into a binary message
                msg = ustruct.pack("<fffffBB", lat, lon, alt, num_sats, sog, track_angle, num_sats, gps_error)
                # Send the message through the HC-12 module
                uart_hc12.write(msg)
            # Wait for some time before reading the next GPS data
            utime.sleep(0.5)
        except Exception as e:
            print("Error reading GPS data:", e)
