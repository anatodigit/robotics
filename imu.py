#http://davekopp.weebly.com/minimu-inertial-measurement-unit-driver-for-raspberry-pi-lsm303d-and-l3gd20h.html
#This Python 2 program reads the data from an LSM303D and an L3GD20H which are both attached to the I2C bus of a Raspberry Pi.
#Both can be purchased as a unit from Pololu as their MinIMU-9 v3 Gyro, Accelerometer, and Compass  product.
#
#First follow the procedure to enable I2C on R-Pi.
#1. Add the lines "ic2-bcm2708" and "i2c-dev" to the file /etc/modules
#2. Comment out the line "blacklist ic2-bcm2708" (with a #) in the file /etc/modprobe.d/raspi-blacklist.conf
#3. Install I2C utility (including smbus) with the command "apt-get install python-smbus i2c-tools"
#4. Connect the I2C device to the SDA and SCL pins of the Raspberry Pi and detect it using the command "i2cdetect -y 1".  It should show up as 1D (typically) or 1E (if the jumper is set).

import time, math

class MyIMU(object):
    b = False
    busNum = 1
    ## LSM303D Registers --------------------------------------------------------------
    LSM = 0x1d #Device I2C slave address
    
    LSM_WHOAMI_ADDRESS = 0x0F
    LSM_WHOAMI_CONTENTS = 0b1001001 #Device self-id
    
    #Control register addresses -- from LSM303D datasheet
    
    LSM_CTRL_0 = 0x1F #General settings
    LSM_CTRL_1 = 0x20 #Turns on accelerometer and configures data rate
    LSM_CTRL_2 = 0x21 #Self test accelerometer, anti-aliasing accel filter
    LSM_CTRL_3 = 0x22 #Interrupts
    LSM_CTRL_4 = 0x23 #Interrupts
    LSM_CTRL_5 = 0x24 #Turns on temperature sensor
    LSM_CTRL_6 = 0x25 #Magnetic resolution selection, data rate config
    LSM_CTRL_7 = 0x26 #Turns on magnetometer and adjusts mode
    
    #Registers holding twos-complemented MSB and LSB of magnetometer readings -- from LSM303D datasheet
    LSM_MAG_X_LSB = 0x08 # x
    LSM_MAG_X_MSB = 0x09
    LSM_MAG_Y_LSB = 0x0A # y
    LSM_MAG_Y_MSB = 0x0B
    LSM_MAG_Z_LSB = 0x0C # z
    LSM_MAG_Z_MSB = 0x0D
    
    #Registers holding twos-complemented MSB and LSB of magnetometer readings -- from LSM303D datasheet
    LSM_ACC_X_LSB = 0x28 # x
    LSM_ACC_X_MSB = 0x29
    LSM_ACC_Y_LSB = 0x2A # y
    LSM_ACC_Y_MSB = 0x2B
    LSM_ACC_Z_LSB = 0x2C # z
    LSM_ACC_Z_MSB = 0x2D
    
    #Registers holding 12-bit right justified, twos-complemented temperature data -- from LSM303D datasheet
    LSM_TEMP_MSB = 0x05
    LSM_TEMP_LSB = 0x06
    
    # L3GD20H registers ----------------------------------------------------
    
    LGD = 0x6b #Device I2C slave address
    LGD_WHOAMI_ADDRESS = 0x0F
    LGD_WHOAMI_CONTENTS = 0b11010111 #Device self-id
    
    LGD_CTRL_1 = 0x20 #turns on gyro
    LGD_CTRL_2 = 0x21 #can set a high-pass filter for gyro
    LGD_CTRL_3 = 0x22
    LGD_CTRL_4 = 0x23
    LGD_CTRL_5 = 0x24
    LGD_CTRL_6 = 0x25
    
    LGD_TEMP = 0x26
    
    #Registers holding gyroscope readings
    LGD_GYRO_X_LSB = 0x28
    LGD_GYRO_X_MSB = 0x29
    LGD_GYRO_Y_LSB = 0x2A
    LGD_GYRO_Y_MSB = 0x2B
    LGD_GYRO_Z_LSB = 0x2C
    LGD_GYRO_Z_MSB = 0x2D
    
    #Kalman filter variables
    Q_angle = 0.02
    Q_gyro = 0.0015
    R_angle = 0.005
    y_bias = 0.0
    x_bias = 0.0
    XP_00 = 0.0
    XP_01 = 0.0
    XP_10 = 0.0
    XP_11 = 0.0
    YP_00 = 0.0
    YP_01 = 0.0
    YP_10 = 0.0
    YP_11 = 0.0
    KFangleX = 0.0
    KFangleY = 0.0


    def __init__(self):
        from smbus import SMBus
        self.b = SMBus(self.busNum)
        self.detectTest()
        #Set up the chips for reading  ----------------------
        self.b.write_byte_data(self.LSM, self.LSM_CTRL_1, 0b1010111) # enable accelerometer, 50 hz sampling
        self.b.write_byte_data(self.LSM, self.LSM_CTRL_2, 0x00) #set +/- 2g full scale
        self.b.write_byte_data(self.LSM, self.LSM_CTRL_5, 0b01100100) #high resolution mode, thermometer off, 6.25hz ODR
        self.b.write_byte_data(self.LSM, self.LSM_CTRL_6, 0b00100000) # set +/- 4 gauss full scale
        self.b.write_byte_data(self.LSM, self.LSM_CTRL_7, 0x00) #get magnetometer out of low power mode
        self.b.write_byte_data(self.LGD, self.LGD_CTRL_1, 0x0F) #turn on gyro and set to normal mode
        #Read data from the chips ----------------------
        #while True:
        #    time.sleep(0.5)
        #    print self.readSensors()
        self.newRead()
    
    
    def twos_comp_combine(self, msb, lsb):
        twos_comp = 256*msb + lsb
        if twos_comp >= 32768:
            return twos_comp - 65536
        else:
            return twos_comp
            
    def detectTest(self):
        #Ensure chip is detected properly on the bus ----------------------
        if self.b.read_byte_data(self.LSM, self.LSM_WHOAMI_ADDRESS) == self.LSM_WHOAMI_CONTENTS:
            print 'LSM303D detected successfully on I2C bus '+str(self.busNum)+'.'
        else:
            print 'No LSM303D detected on bus on I2C bus '+str(self.busNum)+'.'
        
        if self.b.read_byte_data(self.LGD, self.LGD_WHOAMI_ADDRESS) == self.LGD_WHOAMI_CONTENTS:
            print 'L3GD20H detected successfully on I2C bus '+str(self.busNum)+'.'
        else:
            print 'No L3GD20H detected on bus on I2C bus '+str(self.busNum)+'.'
    
    
    def readSensors(self):
        data = (self.readMagX(), self.readMagY(), self.readMagZ(), self.readAccX(), self.readAccY(), self.readAccZ(), self.readGyroX(), self.readGyroY(), self.readGyroZ())
        return data


    def readMagX(self):
        return self.twos_comp_combine(self.b.read_byte_data(self.LSM, self.LSM_MAG_X_MSB), self.b.read_byte_data(self.LSM, self.LSM_MAG_X_LSB))
        
    def readMagY(self):
        return self.twos_comp_combine(self.b.read_byte_data(self.LSM, self.LSM_MAG_Y_MSB), self.b.read_byte_data(self.LSM, self.LSM_MAG_Y_LSB))
        
    def readMagZ(self):
        return self.twos_comp_combine(self.b.read_byte_data(self.LSM, self.LSM_MAG_Z_MSB), self.b.read_byte_data(self.LSM, self.LSM_MAG_Z_LSB))
        
    def readAccX(self):
        return self.twos_comp_combine(self.b.read_byte_data(self.LSM, self.LSM_ACC_X_MSB), self.b.read_byte_data(self.LSM, self.LSM_ACC_X_LSB))
    
    def readAccY(self):
        return self.twos_comp_combine(self.b.read_byte_data(self.LSM, self.LSM_ACC_Y_MSB), self.b.read_byte_data(self.LSM, self.LSM_ACC_Y_LSB))
   
    def readAccZ(self):
        return self.twos_comp_combine(self.b.read_byte_data(self.LSM, self.LSM_ACC_Z_MSB), self.b.read_byte_data(self.LSM, self.LSM_ACC_Z_LSB))
        
    def readGyroX(self):
        return self.twos_comp_combine(self.b.read_byte_data(self.LGD, self.LGD_GYRO_X_MSB), self.b.read_byte_data(self.LGD, self.LGD_GYRO_X_LSB))
        
    def readGyroY(self):
        return self.twos_comp_combine(self.b.read_byte_data(self.LGD, self.LGD_GYRO_Y_MSB), self.b.read_byte_data(self.LGD, self.LGD_GYRO_Y_LSB))
        
    def readGyroZ(self):
        return self.twos_comp_combine(self.b.read_byte_data(self.LGD, self.LGD_GYRO_Z_MSB), self.b.read_byte_data(self.LGD, self.LGD_GYRO_Z_LSB))
    
        
    def newRead(self):
        import datetime
        
        RAD_TO_DEG = 57.29578
        M_PI = 3.14159265358979323846
        G_GAIN = 0.070  # [deg/s/LSB]  If you change the dps for gyro, you need to update this value accordingly
        AA =  0.40      # Complementary filter constant

        gyroXangle = 0.0
        gyroYangle = 0.0
        gyroZangle = 0.0
        CFangleX = 0.0
        CFangleY = 0.0
        kalmanX = 0.0
        kalmanY = 0.0
        
        a = datetime.datetime.now()
        
        while True:
            #Read the accelerometer,gyroscope and magnetometer values
            ACCx = self.readAccX()
            ACCy = self.readAccY()
            ACCz = self.readAccZ()
            GYRx = self.readGyroX()
            GYRy = self.readGyroY()
            GYRz = self.readGyroZ()
            MAGx = self.readMagX()
            MAGy = self.readMagY()
            MAGz = self.readMagZ()
            
            ##Calculate loop Period(LP). How long between Gyro Reads
            b = datetime.datetime.now() - a
            a = datetime.datetime.now()
            LP = b.microseconds/(1000000*1.0)
            print "Loop Time | %5.2f|" % ( LP ),
            
            
            #Convert Gyro raw to degrees per second
            rate_gyr_x =  GYRx * G_GAIN
            rate_gyr_y =  GYRy * G_GAIN
            rate_gyr_z =  GYRz * G_GAIN
            
            
            #Calculate the angles from the gyro. 
            gyroXangle+=rate_gyr_x*LP
            gyroYangle+=rate_gyr_y*LP
            gyroZangle+=rate_gyr_z*LP
            
            
            ##Convert Accelerometer values to degrees
            AccXangle =  (math.atan2(ACCy,ACCz)+M_PI)*RAD_TO_DEG
            AccYangle =  (math.atan2(ACCz,ACCx)+M_PI)*RAD_TO_DEG
            
            
            ####################################################################
            ######################Correct rotation value########################
            ####################################################################
            #Change the rotation value of the accelerometer to -/+ 180 and
            	#move the Y axis '0' point to up.
            	#
            	#Two different pieces of code are used depending on how your IMU is mounted.
            #If IMU is up the correct way, Skull logo is facing down, Use these lines
            AccXangle -= 180.0
            if AccYangle > 90:
            	AccYangle -= 270.0
            else:
            	AccYangle += 90.0
            #
            #
            #
            #
            #If IMU is upside down E.g Skull logo is facing up;
            #if AccXangle >180:
            	#        AccXangle -= 360.0
            #AccYangle-=90
            #if (AccYangle >180):
            	#        AccYangle -= 360.0
            ############################ END ##################################
            
            
            #Complementary filter used to combine the accelerometer and gyro values.
            CFangleX=AA*(CFangleX+rate_gyr_x*LP) +(1 - AA) * AccXangle
            CFangleY=AA*(CFangleY+rate_gyr_y*LP) +(1 - AA) * AccYangle
            
            #Kalman filter used to combine the accelerometer and gyro values.
            kalmanY = self.kalmanFilterY(AccYangle, rate_gyr_y,LP)
            kalmanX = self.kalmanFilterX(AccXangle, rate_gyr_x,LP)
            
            
            ####################################################################
            ############################MAG direction ##########################
            ####################################################################
            #If IMU is upside down, then use this line.  It isnt needed if the
            # IMU is the correct way up
            #MAGy = -MAGy
            #
            ############################ END ##################################
            
            
            #Calculate heading
            heading = 180 * math.atan2(MAGy,MAGx)/M_PI
            
            #Only have our heading between 0 and 360
            if heading < 0:
             	heading += 360
            
            
            #Normalize accelerometer raw values.
            accXnorm = ACCx/math.sqrt(ACCx * ACCx + ACCy * ACCy + ACCz * ACCz)
            accYnorm = ACCy/math.sqrt(ACCx * ACCx + ACCy * ACCy + ACCz * ACCz)
            
            ####################################################################
            ###################Calculate pitch and roll#########################
            ####################################################################
            #Us these two lines when the IMU is up the right way. Skull logo is facing down
            pitch = math.asin(accXnorm)
            roll = -math.asin(accYnorm/math.cos(pitch))
            #
            #Us these four lines when the IMU is upside down. Skull logo is facing up
            #accXnorm = -accXnorm				#flip Xnorm as the IMU is upside down
            #accYnorm = -accYnorm				#flip Ynorm as the IMU is upside down
            #pitch = math.asin(accXnorm)
            #roll = math.asin(accYnorm/math.cos(pitch))
            #
            ############################ END ##################################
            
            #Calculate the new tilt compensated values
            magXcomp = MAGx*math.cos(pitch)+MAGz*math.sin(pitch)
            magYcomp = MAGx*math.sin(roll)*math.sin(pitch)+MAGy*math.cos(roll)-MAGz*math.sin(roll)*math.cos(pitch)
            
            #Calculate tilt compensated heading
            tiltCompensatedHeading = 180 * math.atan2(magYcomp,magXcomp)/M_PI
        
            if tiltCompensatedHeading < 0:
                    tiltCompensatedHeading += 360
            
            
            
            if 1:			#Change to '0' to stop showing the angles from the accelerometer
            	print ("\033[1;34;40mACCX Angle %5.2f ACCY Angle %5.2f  \033[0m  " % (AccXangle, AccYangle)),
            
            if 1:			#Change to '0' to stop  showing the angles from the gyro
            	print ("\033[1;31;40m\tGRYX Angle %5.2f  GYRY Angle %5.2f  GYRZ Angle %5.2f" % (gyroXangle,gyroYangle,gyroZangle)),
            
            if 1:			#Change to '0' to stop  showing the angles from the complementary filter
            	print ("\033[1;35;40m   \tCFangleX Angle %5.2f \033[1;36;40m  CFangleY Angle %5.2f \33[1;32;40m" % (CFangleX,CFangleY)),
            	
            if 1:			#Change to '0' to stop  showing the heading
            	print ("HEADING  %5.2f \33[1;37;40m tiltCompensatedHeading %5.2f" % (heading,tiltCompensatedHeading)),
            	
            if 1:			#Change to '0' to stop  showing the angles from the Kalmin filter
            	print ("\033[1;31;40m kalmanX %5.2f  \033[1;35;40m kalmanY %5.2f  " % (kalmanX,kalmanY))
            
            
            #slow program down a bit, makes the output more readable
            time.sleep(0.03)   
            
    def kalmanFilterY (self, accAngle, gyroRate, DT):
    	y=0.0
    	S=0.0
        '''
    	global KFangleY
    	global Q_angle
    	global Q_gyro
    	global y_bias
    	global XP_00
    	global XP_01
    	global XP_10
    	global XP_11
    	global YP_00
    	global YP_01
    	global YP_10
    	global YP_11
        '''
    
    	self.KFangleY = self.KFangleY + DT * (gyroRate - self.y_bias)
    
    	self.YP_00 = self.YP_00 + ( - DT * (self.YP_10 + self.YP_01) + self.Q_angle * DT )
    	self.YP_01 = self.YP_01 + ( - DT * self.YP_11 )
    	self.YP_10 = self.YP_10 + ( - DT * self.YP_11 )
    	self.YP_11 = self.YP_11 + ( + self.Q_gyro * DT )
    
    	y = accAngle - self.KFangleY
    	S = self.YP_00 + self.R_angle
    	K_0 = self.YP_00 / S
    	K_1 = self.YP_10 / S
    	
    	self.KFangleY = self.KFangleY + ( K_0 * y )
    	self.y_bias = self.y_bias + ( K_1 * y )
    	
    	self.YP_00 = self.YP_00 - ( K_0 * self.YP_00 )
    	self.YP_01 = self.YP_01 - ( K_0 * self.YP_01 )
    	self.YP_10 = self.YP_10 - ( K_1 * self.YP_00 )
    	self.YP_11 = self.YP_11 - ( K_1 * self.YP_01 )
    	
    	return self.KFangleY
    
    def kalmanFilterX (self, accAngle, gyroRate, DT):
    	x=0.0
    	S=0.0
        '''
    	global KFangleX
    	global Q_angle
    	global Q_gyro
    	global x_bias
    	global XP_00
    	global XP_01
    	global XP_10
    	global XP_11
    	global YP_00
    	global YP_01
    	global YP_10
    	global YP_11
        '''
    
    	self.KFangleX = self.KFangleX + DT * (gyroRate - self.x_bias)
    
    	self.YP_00 = self.YP_00 + ( - DT * (self.YP_10 + self.YP_01) + self.Q_angle * DT )
    	self.YP_01 = self.YP_01 + ( - DT * self.YP_11 )
    	self.YP_10 = self.YP_10 + ( - DT * self.YP_11 )
    	self.YP_11 = self.YP_11 + ( + self.Q_gyro * DT )
    
    	x = accAngle - self.KFangleX
    	S = self.YP_00 + self.R_angle
    	K_0 = self.YP_00 / S
    	K_1 = self.YP_10 / S
    	
    	self.KFangleX = self.KFangleX + ( K_0 * x )
    	self.x_bias = self.x_bias + ( K_1 * x )
    	
    	self.YP_00 = self.YP_00 - ( K_0 * self.YP_00 )
    	self.YP_01 = self.YP_01 - ( K_0 * self.YP_01 )
    	self.YP_10 = self.YP_10 - ( K_1 * self.YP_00 )
    	self.YP_11 = self.YP_11 - ( K_1 * self.YP_01 )
    	
    	return self.KFangleX