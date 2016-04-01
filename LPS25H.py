#https://www.pololu.com/file/download/LPS25H.pdf?file_id=0J761
from smbus import SMBus
busNum = 1
b = SMBus(busNum)
LSM = 0x5c
LSM_WHOAMI = 0b1011101 #Device self-id

print 'test:'+str(b.read_byte_data(LSM, 0x0f))


if b.read_byte_data(LSM, 0x0f) == LSM_WHOAMI:
    print 'LPS25H detected successfully.'
else:
    print 'No LPS25H detected on bus '+str(busNum)+'.'