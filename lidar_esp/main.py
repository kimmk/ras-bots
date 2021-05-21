import micropython
import machine
import network
import socket
import time
from machine import Pin, I2C, Timer
from vl53l1x import VL53L1X

# setup ap
ap = network.WLAN(network.AP_IF)
ap.active(True)
ap.config(essid='lidar_esp', password='hyvinsalainen', authmode=network.AUTH_WPA2_PSK, channel=11)

time.sleep(1)

# connect to an ap
wlan = network.WLAN(network.STA_IF)
wlan.active(True)
wlan.scan()
wlan.connect('LEGOS', '12345678')
timeout = time.time() + 20
while not wlan.isconnected() and time.time() < timeout:
    time.sleep(0.1)

# setup lidar
i2c = I2C(scl=Pin(5), sda=Pin(4), freq=100000)
lidar = VL53L1X(i2c)

# create udp broadcast socket (assumes subnet mask 255.255.255.0)
net = wlan
ip = net.ifconfig()[0].split(".")
ip = ".".join(ip[0:3])+".255"
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
print("broadcasting to {}".format(ip))

# broadcasts lidar range data
def advertise_range(t):
    if net.active():
        range = lidar.read()
        sock.sendto(b"{}".format(range), (ip, 7171))

tim = Timer(-1)
tim.init(freq=30, mode=Timer.PERIODIC, callback=advertise_range)
