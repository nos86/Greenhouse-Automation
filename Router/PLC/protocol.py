import serial
import logging
import re

class exceptions:
    class NotSupportedCommand(Exception):
        pass
    class ResponseMalformed(Exception):
        pass
    class WrongChecksum(Exception):
        pass

class Serial(serial.Serial):
    STX = b"\x02" #Start of text
    ETX = b'\x03' #End of text
    EOT = b'\x04' #End of transmission
    ENQ = b'\x05' #Enquiry
    ACK = b'\x06' #Acknowledge
    LF = b'\x0A'  #Line Feed
    CL = b'\x0C'  #Clear
    CR = b'\x0D'  #Carrier Return
    NAK = b'\x15' #Not Acknowledge

    register_regex = re.compile('^(?P<type>[A-Z]+)(?P<register>[0-9]+)$')
    register_offset = {
        'D': {'offset': 0x4000, 'size': 2},
        'C': {'offset': 0x0A00, 'size': 2},
        'T': {'offset': 0x1000, 'size': 2},
        'M': {'offset': 0x0000, 'size': 2},
        'X': {'offset': 0x1200, 'size': 2},
        'Y': {'offset': 0x0C00, 'size': 1}
    }

    def __init__(self, port, baudrate=9600, bytesize = serial.SEVENBITS, 
                parity = serial.PARITY_EVEN, stopbits = serial.STOPBITS_ONE, 
                timeout = 5, logging_level = logging.INFO):
        serial.Serial.__init__(self, port, baudrate, bytesize, parity, stopbits, timeout)
        self.logger = logging.getLogger(self.__class__.__name__)
        self.logger.setLevel(logging_level)
        self.handshake = lambda: self.write(self.ENQ, rawCommand=True) == self.ACK
        if not self.handshake():
            self.logger.error("Unable to handshake with plc")

    def write(self, payload, rawCommand = False, flushBefore = True):
        if flushBefore:
            self.flushOutput()
            self.flushInput()
        if rawCommand == False:
            payload = self.STX + payload.encode() + self.ETX + self.calculateChecksum(payload).encode()
        elif isinstance(payload, str):
            payload = payload.encode()
        self.logger.debug("< [{}] {} bytes".format(''.join([i if ord(i) > 15 else '.' for i in payload]), len(payload)))
        super(Serial, self).write(payload)
        response = self.read()
        return response

    def read(self):
        #Look for starting character
        data = super(Serial, self).read(1)
        if data == self.ACK:
            self.logger.debug("> ACK")
            return self.ACK
        elif data == self.STX: #message
            char = super(Serial, self).read(1)
            while char != self.ETX and char is not None:
                data += char
                char = super(Serial, self).read(1)
            if char == self.ETX:
                data += char + super(Serial, self).read(2)
                self.logger.debug("> [{}] {} bytes".format(''.join([i if ord(i) > 15 else '.' for i in data]), len(data)))
                message = data[1:-3].decode()
                if data[-2:] != self.calculateChecksum(message).encode():
                    raise exceptions.WrongChecksum("Expected: {} | Received: {}".format(self.calculateChecksum(message), data[-2:]))
                return message
            else:
                self.logger.debug("> [{}] {} bytes - Timeout of communication".format(''.join([i if ord(i) > 15 else '.' for i in data]), len(data)))
                raise exceptions.ResponseMalformed()
        elif data == '':
            self.logger.error("No answer from PLC")
            return None
        else:
            self.logger.error("Received unknown character: 0x{:02X}".format(int(data,16)))
            return data


    def calculateChecksum(self, payload):
        _sum = 3
        for ch in payload:
            _sum = _sum + ord(ch)
        return "{:02X}".format(_sum & 0xff)
