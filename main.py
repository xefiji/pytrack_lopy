from network import Sigfox
from network import WLAN
from L76GNSS import L76GNSS
from L76GNSSV4 import L76GNSS
from pytrack import Pytrack
from machine import I2C

from micropygps import MicropyGPS

import socket
import machine
import utime
import pycom
import network
import os
import time
import utime
import struct
import gc
import sys

# GPS
l76 = 1
micropygps = 2 # https://github.com/inmcm/micropyGPS
l762 = 3 # https://github.com/andrethemac/L76GLNSV4
active_gps = l762

# vars
sent, searched = 0, 0 # number of time coords have been sent and searched (l76)
max_search = 120 # max amount of gps lookup when coords not found (l76) or max amount of time gps lookup when coords not found (micropygps)
sleep = 660 # seconds to sleep after successfull send (11 minutes)

# pytrack
py = Pytrack()

# gps
if active_gps == micropygps:    
    i2c = machine.I2C(0, I2C.MASTER, pins=('P22', 'P21'))
    GPS_I2CADDR = const(0x10) # write to address of GPS (16)
    raw = bytearray(1)
    i2c.writeto(GPS_I2CADDR, raw)
    gps = MicropyGPS(0, location_formatting='dd')
elif active_gps == l76:
    gps = L76GNSS(py, timeout=30)
elif active_gps == l762:
    gps = L76GNSS(pytrack=py)
    gps.setAlwaysOn()

# network
rtc = machine.RTC()
wlan = WLAN(mode=WLAN.STA)
ssid = 'Blacksnakes'
wpa = 'mescaline'

# buttons
button = 'P14'
pin = machine.Pin(button, mode=machine.Pin.IN, pull=machine.Pin.PULL_UP)

# Functions

def pin_handler(arg):
    print("Going for deepsleep...")
    pycom.heartbeat(False)
    pycom.rgbled(0xeb0e0e) # red
    time.sleep(0.3)
    machine.deepsleep()

def send(sigfox_socket, lat, lon, alt, speed):    
    try:
        p = struct.pack('f', float(lat)) + struct.pack('f', float(lon)) + struct.pack('f', float(alt))        
        sigfox_socket.send(p)
        print("Sent {}bytes - #{}: {} - {} - {}".format(struct.calcsize('fff'), sent, coord, rtc.now(), gc.mem_free()))        
    except:
        print("Unexpected error:", sys.exc_info()) 

def wifi_setup():
    nets = wlan.scan()
    for net in nets:
        if net.ssid == ssid:
            print('Network found; connecting to {}'.format(ssid))
            wlan.connect(net.ssid, auth=(net.sec, wpa), timeout=5000)
            while not wlan.isconnected():
                machine.idle() # save power while waiting
            print('Connected to {}'.format(ssid))
            break

def blink():
    # startup blink
    pycom.heartbeat(False)
    pycom.rgbled(0x007f00) # green
    time.sleep(0.3)
    pycom.rgbled(0x7f7f00) # yellow
    time.sleep(0.3)
    pycom.rgbled(0x007f00) # green
    time.sleep(0.3)
    pycom.heartbeat(True)

def rtc_setup():
    # setup rtc
    rtc.ntp_sync("pool.ntp.org")
    utime.sleep_ms(750)
    utime.timezone(7200)
    print('Adjusted from UTC to EST timezone', utime.localtime(), '\n')

def sigfox_setup():
    # setup sigfox
    sigfox = Sigfox(mode=Sigfox.SIGFOX, rcz=Sigfox.RCZ1)
    sigfox_socket = socket.socket(socket.AF_SIGFOX, socket.SOCK_RAW)
    sigfox_socket.setblocking(True)
    sigfox_socket.setsockopt(socket.SOL_SIGFOX, socket.SO_RX, False)
    return sigfox_socket

def go_sleep(duration):
    print("sleeping {}secs".format(duration))
    py.setup_sleep(duration)
    py.go_to_sleep(gps=True)

def read_coords(active_gps):
    if active_gps == micropygps:
        # return (45.181881, 5.791920, 227.2, 5.67) # for tests
        # read from gps via I2C
        raw = i2c.readfrom(GPS_I2CADDR, 16)        
        # feed gps object
        for b in raw:
            sentence = gps.update(chr(b))            
            if sentence is not None and gps.satellite_data_updated() and gps.valid:    
                return (gps.latitude[0], gps.longitude[0], gps.altitude, gps.speed[2])
    elif active_gps == l76:
        coord = gps.coordinates() #coord = (45.181881, 5.791920) # for tests    
        if not (coord == (None, None)):
            return (coord[0], coord[1], 0.0, 0.0)
    elif active_gps == l762:
        location = gps.get_location()
        if location["latitude"] is not None and location["longitude"] is not None:
            return (location["latitude"], location["longitude"], location["altitude"], 0.0)
        pass

    return (None, None, None, None)

# Function calls
print("Wakeup reason: " + str(py.get_wake_reason()))
blink()
pin.callback(machine.Pin.IRQ_FALLING | machine.Pin.IRQ_RISING, pin_handler)
time.sleep(2)
gc.enable()
wifi_setup()
rtc_setup()
sigfox_socket = sigfox_setup()

# Runtime
print("RUNNING WITH GPS AND SIGFOX")  
time.sleep(3) # wait for time to be synced
start_time = time.time() # will be substracted for elapsed time (micropygps)

while (True):
    coord = read_coords(active_gps)    
    if not (coord == (None, None, None, None)):
        send(sigfox_socket, coord[0], coord[1], coord[2], coord[3]) # actual sigfox send
        time.sleep(1)
        sent += 1
        pycom.heartbeat(False)
        pycom.rgbled(0x007f00) # green
        time.sleep(0.3)
        
        go_sleep(sleep)

    if active_gps == micropygps:
        elapsed_time = time.time() - start_time
        if elapsed_time >= max_search:
            pycom.heartbeat(False)
            pycom.rgbled(0xeb0e0e) # red
            go_sleep(15) #15 secs before retry
            break
    else:
        print("Searching coords {}...".format(searched))
        searched += 1

        if searched >= max_search:
            pycom.heartbeat(False)
            pycom.rgbled(0xeb0e0e) # red
            go_sleep(15) #15 secs before retry
            break

    """
    if (sent == 0) or (not coord == (None, None) and diff > delay):
        send(coord[0], coord[1])    
        init_timer = time.time()
        sent += 1

        print("sleeping {}secs".format(sleep))
        # py.setup_sleep(sleep)
        # py.go_to_sleep()
    """


"""
If Wakeup == acelerometer
 last move = check last move
 if last move > now - delay
    Start
else:
    sleep
"""

"""
If Wakeup == timer
 last move = check last move
 if last move < now - 15min
    deepsleep
"""