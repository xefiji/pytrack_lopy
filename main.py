from network import Sigfox
from network import WLAN
from L76GNSS import L76GNSS
from pytrack import Pytrack

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

sent, searched = 0, 0 # number of time coords have been sent and searched
max_search = 120 # max amount of gps lookup when coords not found
sleep = 660 # seconds to sleep (11 minutes)
py = Pytrack()
l76 = L76GNSS(py, timeout=30)
rtc = machine.RTC()
wlan = WLAN(mode=WLAN.STA)
ssid = 'xxx'
wpa = 'xxx'
button = 'P14'
pin = machine.Pin(button, mode=machine.Pin.IN, pull=machine.Pin.PULL_UP)

def pin_handler(arg):
    print("Going for deepsleep...")
    pycom.heartbeat(False)
    pycom.rgbled(0xeb0e0e) # red
    time.sleep(0.3)
    machine.deepsleep()

def send(sigfox_socket, lat, lon):
    try:
        p = struct.pack('f', float(lat)) + struct.pack('f', float(lon))    
        sigfox_socket.send(p)
        print("Sent {}bytes - #{}: {} - {} - {}".format(struct.calcsize('ff'), sent, coord, rtc.now(), gc.mem_free()))        
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

# function calls
print("Wakeup reason: " + str(py.get_wake_reason()))
blink()
pin.callback(machine.Pin.IRQ_FALLING | machine.Pin.IRQ_RISING, pin_handler)
time.sleep(2)
gc.enable()
wifi_setup()
rtc_setup()
sigfox_socket = sigfox_setup()

print("RUNNING WITH GPS AND SIGFOX")       

while (True):
    coord = l76.coordinates() #coord = (45.181881, 5.791920) # for tests    
    if not (coord == (None, None)):
        send(sigfox_socket, coord[0], coord[1]) # actual sigfox send
        time.sleep(1)
        sent += 1
        pycom.heartbeat(False)
        pycom.rgbled(0x007f00) # green
        time.sleep(0.3)
        
        go_sleep(sleep)

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