#https://www.pololu.com/file/download/LPS25H.pdf?file_id=0J761
from smbus import SMBus
busNum = 1
b = SMBus(busNum)
PTS = 0x5d #Device I2C slave address
PTS_WHOAMI = 0b1011101 #Device self-id

(chip_id, version) = b.read_i2c_block_data(PTS, 0xD0, 2)
print "Chip Id:", chip_id, "Version:", version


if b.read_byte_data(PTS, 0x0f) == PTS_WHOAMI:
    print 'LPS25H detected successfully.'
else:
    print 'No LPS25H detected on bus '+str(busNum)+'.'