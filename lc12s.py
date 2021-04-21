#!/usr/bin/env python
import serial
from serial import Serial
import RPi.GPIO as GPIO
import time
from struct import pack, unpack
from functools import partial
import sys
import argparse
from tqdm import tqdm
parser = argparse.ArgumentParser()
parser.add_argument("--target-frequency", help="Target broadcast frequency (how often does target broadcast?) [hz]",type=float,default=1)
parser.add_argument("--cs-pin", help="cs pin",default=38,type=int)
parser.add_argument("--set-pin", help="set pin",default=40,type=int)
parser.add_argument("--led-pin", help="led pin",default=11,type=int)
parser.add_argument("--mode", help="mode [rw,listen]",default="rw",type=str)
parser.add_argument("--channel", help="RF channel as hex string",default="0x0A",type=str)
parser.add_argument("--net", help="network id",default="0x0C",type=str)
args = parser.parse_args()


GPIO.setmode(GPIO.BOARD)

GPIO.setup(args.cs_pin, GPIO.OUT)
GPIO.setup(args.set_pin, GPIO.OUT)


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

msg_format='BBHHBBBBBBHBBBB'
msg_fields=[
        'h1','h2',
        'selfID','netID','h3','RFpower','h4','serial','h5','RFchannel','h6','h7','len','h8','checksum']

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
        #msg_binary=b"".join([ pack(f,v) for f,v in zip(msg_format,msg) ])
        msg_binary=pack(msg_format,*msg)
        msg[-1]=sum(msg_binary) & 0xFF # add the checksum
        
        #msg_binary=b"".join([ pack(f,v) for f,v in zip(msg_format,msg) ])
        msg_binary=pack(msg_format,*msg)
        return msg_binary



    @classmethod
    def from_msg(cls,msg):
        msg_values=unpack(msg_format,msg)
        d={ msg_fields[i]:msg_values[i] for i in range(len(msg_values)) }
        m=cls(d['selfID'],d['netID'],d['RFpower'],d['serial'],d['RFchannel'])
        return m


    def __str__(self):
        return "selfID:%x,netID:%x,RFpower:%x,serial:%x,RFchannel:%x " % (self.module_id,self.network_id,self.rf_transmit_power,self.serial_rate,self.rf_channel) + self.binary().hex()



#lets make sure we can recreate the one in the docs        
m1=lc12s_msg(0x00,0x00,0x00,0x04,0x0A)
assert(m1.binary()==0xaa5a0000000000000004000a000000120024.to_bytes(18,byteorder='big'))

#m1=lc12s_msg(0x00,0x01,0x00,0x04,0x0A) #change the network ID

serial_port_fn='/dev/serial0' # might need to be /dev/seria01
baudrate=9600

lc12s_serial = serial.Serial(
        port=serial_port_fn, #Replace ttyS0 with ttyAM0 for Pi1,Pi2,Pi0
        baudrate = baudrate,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=max(0.05,1.0/args.target_frequency)
)


GPIO.output(args.set_pin, GPIO.LOW)
time.sleep(0.2)
GPIO.output(args.cs_pin, GPIO.LOW)
time.sleep(0.2)

def set_lc12s(settings,retries=3,max_fails=30):
    GPIO.output(args.set_pin, GPIO.LOW) # set it to setting mode
    lc12s_serial.write(settings.binary()) # write the settings

    #confirm response
    fails=0
    raw_response=lc12s_serial.read()
    while len(raw_response)<18 and fails<max_fails:
        raw_response+=lc12s_serial.read()
        fails+=1
    if fails==max_fails:
        if retries==0:
            print("failed and then failed some more")
            sys.exit(1)
        return set_lc12s(settings,retries-1)
    return lc12s_msg.from_msg(raw_response)

if args.mode=='rw':
    settings=lc12s_msg(0x00,0x00,0x00,0x04,int(args.channel,16))
    self_response=set_lc12s(settings)
    print("Go into read / write mode")
    GPIO.output(args.set_pin, GPIO.HIGH)
    time.sleep(0.2)

    while True:
        lc12s_serial.write( ("HELLO! I AM %d" % self_response.module_id).encode() )
        response=lc12s_serial.read(100)
elif args.mode=='scan-channel':
    current_state={'scanning_channel':-1,
            'reads_on_channel':{}}
    def txrx_callback(current_state,pin):
        current_channel=current_state['scanning_channel']
        if current_channel!=-1:
            if current_channel not in current_state['reads_on_channel']:
                current_state['reads_on_channel'][current_channel]=0
            current_state['reads_on_channel'][current_channel]+=1
    GPIO.setup(args.led_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.add_event_detect(args.led_pin, GPIO.FALLING, callback=partial(txrx_callback,current_state), bouncetime=100)

    #configure
    GPIO.output(args.set_pin, GPIO.LOW)
    #time.sleep(0.1) # give it some time to change from serial mode
    for channel in tqdm(range(128)):
        settings=lc12s_msg(0x00,0x00,0x00,0x04,channel)
        set_lc12s(settings)

        current_state['scanning_channel']=channel
        GPIO.output(args.set_pin, GPIO.HIGH)
        time.sleep(1.0/args.target_frequency)

        current_state['scanning_channel']=-1
        GPIO.output(args.set_pin, GPIO.LOW)
        #time.sleep(0.1) # give it some time to change from serial mode

        if channel in current_state['reads_on_channel']:
            print("Channel 0x%x has %d reads" % (channel,current_state['reads_on_channel'][channel]))  
elif args.mode=='scan-netid':
    active_netids=set()
    #configure
    GPIO.output(args.set_pin, GPIO.LOW)
    for netid in tqdm(range(0xFFFF+1)):
        settings=lc12s_msg(0x00,netid,0x00,0x04,int(args.channel,16))
        lc12s_serial.write(settings.binary())

        raw_response=lc12s_serial.read()
        while len(raw_response)<18:
            raw_response+=lc12s_serial.read()
        response=lc12s_msg.from_msg(raw_response)

        GPIO.output(args.set_pin, GPIO.HIGH)

        response=lc12s_serial.read(100)
        if len(response)>0:
            active_netids.add(netid)
            print("NetID 0x%x is active!" % netid)

        GPIO.output(args.set_pin, GPIO.LOW)
        #time.sleep(0.05) # give it some time to change from serial mode









