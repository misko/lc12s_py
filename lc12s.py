#!/usr/bin/env python
import serial
from serial import Serial
import RPi.GPIO as GPIO
import time
from struct import pack

GPIO.setmode(GPIO.BOARD)
cs_pin = 18 # I think to test parameter setting CS and set can just be both set to low/gnd
set_pin = 31

GPIO.setup(cs_pin, GPIO.OUT)
GPIO.setup(set_pin, GPIO.OUT)


rf_powers={'12dbm':0,
    '10dbm':1,
    '9dbm':2,
    '8dbm':3,
    '6dbm':4,
    '3dbm':5,
    '0dbm':6,
    '-2dbm':7,
    '-5dbm':8,
    '-10dbm':9,
    '-15dbm':10,
    '-20dbm':11,
    '-25dbm':12,
    '-30dbm':13,
    '-35dbm':14}

serial_rates={'600bps':0,
    '1200bps':1,
    '2400bps':2,
    '4800bps':3,
    '9600bps':4,
    '19200bps':5,
    '38400bps':6}


#message format
# byte    values
# 1       0xaa
# 2       0x5a
# 3,4     SelfID (2byte)
# 5,6     NetID (2byte)
# 7       NC 0x00
# 8       RF power
# 9       NC 0x00
# 10      Baud [0-6]
# 11      NC 0x00
# 12      RF channel [0-127]
# 13,14   NC 0x0000
# 15      NC 0x00
# 16      Length (0x12)
# 17      NC 0x00
# 18      Checksum 1byte

msg_format=[
    'B',
    'B',
    'H', # self
    'H', # net
    'B', 
    'B', #rf power
    'B',
    'B', # baud
    'B', 
    'B', # 12 ,  RF chan
    'H', # 13,14
    'B', # 15
    'B', #16
    'B', #17
    'B'] #18


#Data Format :
#Host sends: 0xaa + 0x5a + module ID + network ID (ID must be the same) + 0x00 + RF transmit power + 0x00 + serial port rate
#+ 0x00 + RF channel selection + 0x00 + 0x00 + 0x12 (byte length) + 0x00 + and check byte
#Note: Checksum = all bytes accumulated
#Module answered successfully
#E.g:
#Host sends: AA , 5A , 00 00,  00 00,  00,  RFPower=00 ,00,Buad=04, 00, RFCHan=0A, 00 00, 00, Len=12, 00, Check=24
#Refer to the following data table, the above configuration parameters to set the wireless module is:
#Network ID: 00 00
#RF transmit power: 12dbm
#Serial speed: 9600bps
#RF channel: 10
#And check byte: 24
#Return data: AA 5B 05 21 00 00 00 00 00 04 00 0A 00 00 00 12 00 4B
#Set the node ID is invalid, set the number can be, does not affect the module's real ID, but the last byte and verify to be correct.
#The return parameter will return the module ID. For example, the above module ID is: 0X0521
#Query Parameters Command: AA 5C 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 18 (HEX format)
#Query version number Command: AA 5d 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 07 (HEX format)
#Back: 02 00 01 (HEX format)
#Factory setting: AA 5A 00 00 00 00 00 00 00 04 00 64 00 00 00 12 00 7E (HEX format)


class lc12s_msg:
    def __init__(self,module_id, 
    network_id, 
    rf_transmit_power,
    serial_rate,
    rf_channel,
    ):
        self.module_id=module_id
        self.network_id=network_id
        self.rf_transmit_power=rf_transmit_power
        self.serial_rate=serial_rate
        self.rf_channel=rf_channel


    def binary(self):
        msg=[0xaa,0x5a,self.module_id,
            self.network_id,0x00,self.rf_transmit_power,
            0x00,self.serial_rate,0x00,self.rf_channel,
            0x00,0x00,0x12,0x00,0x00]
        msg_binary=b"".join([ pack(f,v) for f,v in zip(msg_format,msg) ])
        msg[-1]=sum(msg_binary) & 0xFF # add the checksum
        
        msg_binary=b"".join([ pack(f,v) for f,v in zip(msg_format,msg) ])
        return msg_binary

    def __str__(self):
         return self.binary().hex()



#lets make sure we can recreate the one in the docs        
m1=lc12s_msg(0x00,0x00,0x00,0x04,0x0A)
assert(m1.binary()==0xaa5a0000000000000004000a000000120024.to_bytes(18,byteorder='big'))

serial_port_fn='/dev/serial0' # might need to be /dev/seria01
baudrate=9600

lc12s_serial = serial.Serial(
        port=serial_port_fn, #Replace ttyS0 with ttyAM0 for Pi1,Pi2,Pi0
        baudrate = baudrate,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=1
)


GPIO.output(set_pin, GPIO.LOW)
time.sleep(0.2)
GPIO.output(cs_pin, GPIO.LOW)
time.sleep(0.2)


lc12s_serial.write(m1.binary())
print("done write - this might not fail if it didnt work")
time.sleep(0.2)

read=[]
print("waiting for response")
while len(read)<18:
    this_byte=lc12s_serial.read()
    if len(this_byte)==0:
        print("read timeout")
    else:
        read.append(this_byte)
    print("Read a byte!!!!!!",len(read),'in total',read)
