from . import protocol
import re
import logging
import math

class exceptions:
    class NotSupportedCommand(Exception):
        pass

    class NotSupportedRegister(Exception):
        pass

    class ResponseMalformed(Exception):
        pass

    class WrongChecksum(Exception):
        pass

class PLC(protocol.Serial):
    register_regex = re.compile('^(?P<type>[A-Z]+)(?P<register>[0-9]+)$')
    register_offset = {
        'D': {'offset': 0x4000, 'size': 2},
        'C': {'offset': 0x0A00, 'size': 2},
        'T': {'offset': 0x1000, 'size': 2},
        'M': {'offset': 0x0000, 'size': 1},
        'X': {'offset': 0x1200, 'size': 1},
        'Y': {'offset': 0x0C00, 'size': 1}
    }

    def __init__(self, logging_level=logging.INFO):
        protocol.Serial.__init__(self, '/dev/ttyUSB0', logging_level=logging_level)
        self.data_monitor = []
        self.bit_monitor = []
        self.monitor_started = False

    def readAddress(self, address, size):
        #<stx>,X,Y,Y,Y,Y,N,N,<etx>,L,L X = command code (0 = read,1=write) Y = Address (4bytes) N = number of bytes (2bytes) L = sum
        request = '0{0:04X}{1:02X}'.format(address,size)
        response = self.write(payload = request)
        return self.extractResponse(response)

    def getRegisterAddress(self, register, returnSize=False):
        match = self.register_regex.match(register)
        if not match:
            raise ValueError(register)
        data = match.groupdict()
        if data['type'] in self.register_offset.keys():
            address = self.register_offset[data['type']]['offset'] + self.register_offset[data['type']]['size'] * int(data['register'])
            return (address, self.register_offset[data['type']]['size']) if returnSize else address
        else:
            raise exceptions.NotSupportedRegister(register)

    def addRegisterToMonitor(self, register):
        if not self.monitor_started:
            _, size = self.getRegisterAddress(register, returnSize=True) #Check register is well-formed
            if size == 1:
                self.bit_monitor.append(register)
            else:
                self.data_monitor.append(register)
        else:
            self.logger.error("Unable to add registers to monitor while it is active")

    def startMonitor(self):
        if not self.monitor_started:
            data_len = len(self.data_monitor)
            data = "".join([b[2:4]+b[0:2] for b in ["{:04X}".format(self.getRegisterAddress(d)) for d in self.data_monitor]])
            bit_len =  len(self.bit_monitor)
            bit = "".join([b[2:4]+b[0:2] for b in ["{:04X}".format(self.getRegisterAddress(d)) for d in self.bit_monitor]])
            length = (data_len + bit_len) * 2 + 4
            request = "E101400{:02X}{:02X}81{:02X}00{}{}".format(length, data_len, bit_len, data, bit)
            if self.write(request)==self.ACK:
                logging.info("Monitor installed correctly")
                self.monitor_started = True
            else:
                logging.info("Unable to install the monitor")
        else:
            logging.info("Monitor already installed")

    def stopMonitor(self):
        self.monitor_started = False

    def getValueFromMonitor(self, convertNumber=True):
        data_len = len(self.data_monitor)
        byte_len = int(math.ceil(len(self.bit_monitor)/8.0))
        request = "E001790{:02X}".format(data_len * 2 + byte_len)
        response = self.write(request)
        data = {}
        for idx, param in enumerate(self.data_monitor):
            offset = 4 * idx
            code = response[offset:offset+4]
            if convertNumber:
                data[param] = int(code[2:4] + code[0:2],16)
            else:
                data[param] = code[2:4] + code[0:2]
        for idx, param in enumerate(self.bit_monitor):
            offset = 4 * data_len + 2 * ( idx / 8 ) #Search for right byte
            code = int(response[offset:offset+2])
            data[param] = (code & (1 << ( idx % 8 ) )>0)
        return data

    def readAddress_D(self, address):
        data = self.readAddress(address*2 + 0x1000, 2)
        return int(data[2:4]+data[0:2], 16)

    def updateInputOutputStatus(self):
        data = self.write("E00024010",flushBefore=True)
        self.input = self.extractResponse(data)
        data = self.write("E00018010",flushBefore=True)
        self.output = self.extractResponse(data)
    
    
    
    def updateStatus(self, address):
        pass

    def setOutput(self, pin, state):
        if isinstance(pin, str):
            match = self.regex['output'].match(pin.upper())
            if match is None:
                raise exceptions.NotSupportedCommand()
            payload = ('7' if state>0 else '8') +"{:02X}".format(int(match.groupdict()['pin']))+'0'+ ('1' if match.groupdict()['type'] == 'M' else '5') 
            return self.checkACK(self.write(payload))
        else:
            raise exceptions.NotSupportedCommand()
    
