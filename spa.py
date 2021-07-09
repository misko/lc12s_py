#!/usr/bin/env python
from struct import pack, unpack
import crc16
import time
from struct import pack, unpack
from functools import partial
import argparse


class Spa:
    controller_message_fields_and_lengths=[('command_char',1,'B'),
        ('command',2,'H'),
        ('controller_loading_state',1,'B'),
        ('nc',2,'H'),
        ('crc',2,'H')]
    spa_message_fields_and_lengths=[('command_char',1,'B'),
        ('ncA',1,'B'),
        ('configA',1,'B'), # 128+1 after controller sends on/off
        ('ncB',1,'B'),
        ('configB',1,'B'), 
        ('actualTemp',1,'B'),
        ('ncD',1,'B'),
        ('setTemp',1,'B'),
        ('Data',7,'7s'),
        ('crc',2,'H')]


    commands={ 'on_off' : 0x0001,
           'water_filter': 0x0010,
           'bubbles' : 0x0020,
           'heater': 0x0040,
           'toggle_CF': 0x0002,
           'decrease': 0x0008,
           'increase': 0x0004 } 

    configB={'spa_on':0x1,
            'bubbles_on': 0x10,
            'filter_on': 0x8,
            'heater_on': 0x4}


    spa_message_length=sum([ field[1] for field in spa_message_fields_and_lengths ])
    controller_message_length=sum([ field[1] for field in controller_message_fields_and_lengths ])
    
    def __init__(self,model,channel):
        self.model=model
        self.read_bytes=b''
        self.channel=channel
        self.magic_byte=0x80+channel
        self.controller_loading_state=0

    @classmethod
    def get_crc_index(cls,msg_fields):
        idx=0
        for name,length in msg_fields:
            if name=='crc':
                return idx
            idx+=length

    def parse_message(self,msg_fields_and_lengths,msg):
        byte_string="".join( [ field[2] for field in msg_fields_and_lengths] )
        field_names=[ field[0] for field in msg_fields_and_lengths ]
        field_values=unpack(">"+byte_string,msg)
        d = { field_names[idx] : field_values[idx] for idx in range(len(field_names)) }
        return d

    def parse_controller_message(self,msg):
        d=self.parse_message(self.controller_message_fields_and_lengths,msg)
        self.controller_loading_state=d['controller_loading_state']
        print(d)
        
    def parse_spa_message(self,msg):
        d=self.parse_message(self.spa_message_fields_and_lengths,msg)
        print(d)

    def push_read_bytes(self,data):
        if data==None or len(data)==0:
            return
        self.read_bytes+=data
        while len(self.read_bytes)>0 and self.read_bytes[0]!=self.magic_byte:
            self.read_bytes=self.read_bytes[1:]
        if len(self.read_bytes)>max(self.spa_message_length,self.controller_message_length): # there must be work to do
            #try controller message
            msg_crc=crc16.crc16xmodem(self.read_bytes[:self.controller_message_length-2])
            computed_crc=unpack(">H",self.read_bytes[self.controller_message_length-2:self.controller_message_length])[0]
            if msg_crc==computed_crc:
                print("FOUND MESSAGE controller")
                self.parse_controller_message(self.read_bytes[:self.controller_message_length])
            #try spa message
            msg_crc=crc16.crc16xmodem(self.read_bytes[:self.spa_message_length-2])
            computed_crc=unpack(">H",self.read_bytes[self.spa_message_length-2:self.spa_message_length])[0]
            if msg_crc==computed_crc:
                print("FOUND MESSAGE spa")
                self.parse_spa_message(self.read_bytes[:self.spa_message_length])
            self.read_bytes=self.read_bytes[1:]    

    def add_crc(self,msg):
        msg=msg[:-2]+pack(">H",crc16.crc16xmodem(msg[:-2]))
        return msg

    def create_command_message(self,command):
        pack_string=">"+"".join([ field[2] for field in self.controller_message_fields_and_lengths])
        msg=pack(pack_string,self.magic_byte,
                    command,
                    self.controller_loading_state,
                    0x0,
                    0x0)
        msg=self.add_crc(msg)
        return msg    
        

if __name__=='__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--input-file", help="Input file",type=str,required=True)
    parser.add_argument("--model", help="spa model",default="?",type=str)
    args = parser.parse_args()
    spa=Spa(args.model,channel=0x34)
    print(spa.create_command_message(0x0).hex())
    with open(args.input_file,'rb') as f:
        while True:
            r=f.read(3)
            if len(r)==0:
                break
            spa.push_read_bytes(r)
