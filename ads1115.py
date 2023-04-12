from machine import Pin, I2C
import struct

#ADS1115 register addresses
CONV_REG = const(0x00)
CTRL_REG = const(0x01)
LO_THRES_REG = const(0x02)
HI_THRES_REG = const(0x03)

#Config register bit masks
#All functions that modify Config Register
#apart from setting appropriate bits, also
#clear OS bit. When reading it is usually 1
#(no current conversion). But leaving it and
#writing 1 to Conf Reg would trigger conversion
#in single conversion mode. Thus it has to be cleared.
OS_MASK = const(0x8000)
MUX_MASK = const(0x7000)
PGA_MASK = const(0x0E00)
MODE_MASK = const(0x0100)
DR_MASK = const(0x00E0)
COMP_MODE_MASK = const(0x0010)
ALRT_RDY_POL_MASK = const(0x0008)
COMP_LAT_MASK = const(0x0004)
COMP_QUE_MASK = const(0x0003)

#Config register configuration options
SINGLE_CONV = const(0b1)
CONTINOUS_CONV = const(0b0)
#Input multiplexer configuration
MUX_AIN0_AIN1 = const(0b000)
MUX_AIN0_AIN3 = const(0b001)
MUX_AIN1_AIN3 = const(0b010)
MUX_AIN2_AIN3 = const(0b011)
MUX_AIN0_GND = const(0b100)
MUX_AIN1_GND = const(0b101)
MUX_AIN2_GND = const(0b110)
MUX_AIN3_GND = const(0b111)
#Input amplifier output range
FSR_6V = const(0b000)
FSR_4V = const(0b001)
FSR_2V = const(0b010)
FSR_1V = const(0b011)
FSR_0_5V = const(0b100)
FSR_0_2V = const(0b101)
#Conversion data rate in samples per second (SPS)
DR_8_SPS = const(0b000)
DR_16_SPS = const(0b001)
DR_32_SPS = const(0b010)
DR_64_SPS = const(0b011)
DR_128_SPS = const(0b100)
DR_250_SPS = const(0b101)
DR_475_SPS = const(0b110)
DR_860_SPS = const(0b111)
#Enable/disable conversion ready signal
CONV_RDY_EN = const(1)
CONV_RDY_DIS = const(0)
#Comparator mode of operation: traditional/window
COMP_TRAD = const(0b0)
COMP_WINDOW = const(0b1)
#RDY/ALERT pin active low/high
ALRT_RDY_ACT_LO = const(0b0)
ALRT_RDY_ACT_HI = const(0b1)
#Comparator output latch enable/disable
COMP_LATCH_DIS = const(0b0)
COMP_LATCH_EN = const(0b1)
#Comparator disable or enable with queue length: 1, 2, 4
COMP_MODE_DISABLE = const(0b11)
COMP_MODE_ONE_CONV = const(0b00)
COMP_MODE_TWO_CONV = const(0b01)
COMP_MODE_FOUR_CONV = const(0b10)

class ADS1115:
    #get I2C handle and ADS1115 address
    #configure internal class variables which contain state od the device
    def __init__(self, i2c, addr):
        self._i2c = i2c
        self._addr = addr
    
    def _write_register(self, reg_addr, data):
        tmp = bytearray(2)
        tmp[0] = data >> 8
        tmp[1] = data & 0xff
        self._i2c.writeto_mem(self._addr, reg_addr, tmp)

    def _read_register(self, reg_addr):
        tmp = bytearray(2)
        self._i2c.readfrom_mem_into(self._addr, reg_addr, tmp)
        return (tmp[0] << 8) | tmp[1]
    
    #helper function for setting new threshold values
    #checks if given float value of threshold is inside currently
    #selcected PGA range. If it is not the function will saturate
    #the value, for example: with +-6.144V PGA setting passing
    #7.0 to the function will cause it to return 6.144V
    def _check_theshold_overflow(self, float_val, pga_setting):
        if pga_setting == FSR_6V:
            if float_val > 6.1438125:
                return 6.1438125
            elif float_val < -6.144:
                return -6.144
            else:
                return float_val       
        elif pga_setting == FSR_4V:
            if float_val > 4.095875:
                return 4.095875
            elif float_val < -4.096:
                return -4.096
            else:
                return float_val
        elif pga_setting == FSR_2V:
            if float_val > 2.0479375:
                return 2.0479375
            elif float_val < -2.048:
                return -2.048
            else:
                return float_val
        elif pga_setting == FSR_1V:
            if float_val > 1.02396875:
                return 1.02396875
            elif float_val < -1.024:
                return -1.024
            else:
                return float_val
        elif pga_setting == FSR_0_5V:
            if float_val > 0.511984375:
                return 0.511984375
            elif float_val < -0.512:
                return -0.512
            else:
                return float_val
        elif pga_setting == FSR_0_2V:
            if float_val > 0.2559921875:
                return 0.2559921875
            elif float_val < -0.256:
                return -0.256
            else:
                return float_val
        
    def conversion_mode(self, bits):
        tmp = self._read_register(CTRL_REG)
        tmp &= ~(MODE_MASK | OS_MASK) #clear MODE bit
        tmp |= (bits << 8) #set mode bit
        self._write_register(CTRL_REG, tmp)
          
    def data_rate(self, bits): 
        tmp = self._read_register(CTRL_REG)
        tmp &= ~(DR_MASK | OS_MASK) #clear DR bits
        tmp |= (bits << 5) #set data rate bits
        self._write_register(CTRL_REG, tmp)
        
    def trig_single_conv(self):    
        tmp = self._read_register(CTRL_REG)
        tmp |= OS_MASK #set OS bit
        self._write_register(CTRL_REG, tmp)
    
    # IMPORTANT !!! #
    # After using this function comparator has to be
    # reconfigured: thresholds and queue length are changed
    def conversion_rdy_ctrl(self, en_dis):
        if en_dis == CONV_RDY_EN:
            #set bits as describled in datasheet and enable comparator
            self._write_register(LO_THRES_REG, 0x0000)
            self._write_register(HI_THRES_REG, 0xFFFF)
            self.comparator_enable(COMP_MODE_ONE_CONV)
        elif end_dis == CONV_RDY_DIS:
            self._write_register(LO_THRES_REG, 0xFFFF)
            self._write_register(HI_THRES_REG, 0x0000)
            self.comparator_enable(COMP_MODE_DISABLE)
    
    def multiplexer_config(self, bits):  
        tmp = self._read_register(CTRL_REG)
        tmp &= ~(MUX_MASK | OS_MASK) #clear MUX bits
        tmp |= (bits << 12) #set multiplexer config bits
        self._write_register(CTRL_REG, tmp)
        
    def amplifier_range(self, bits):
        tmp = self._read_register(CTRL_REG)
        tmp &= ~(PGA_MASK | OS_MASK) #clear PGA bits
        tmp |= (bits << 9) #set amplifier config bits
        self._write_register(CTRL_REG, tmp)

    def conversion_read(self):
        data = self._read_register(CONV_REG)
        #get PGA configuration
        pga_conf = self._read_register(CTRL_REG)
        pga_conf &= PGA_MASK #leave only PGA bits
        pga_conf = (pga_conf >> 9) #shift them to the left
        if pga_conf == FSR_6V:
            return data * 0.0001875
        elif pga_conf == FSR_4V:
            return data * 0.000125
        elif pga_conf == FSR_2V:
            return data * 0.0000625
        elif pga_conf == FSR_1V:
            return data * 0.00003125
        elif pga_conf == FSR_0_5V:
            return data * 0.000015625
        elif pga_conf == FSR_0_2V:
            return data * 0.0000078125
        else:
            return 0.0
      
    def conversion_read_raw(self):
        return self._read_register(CONV_REG)
    
    def comparator_enable(self, bits):
        tmp = self._read_register(CTRL_REG)
        tmp &= ~(COMP_QUE_MASK | OS_MASK) #clear comparator config bits
        tmp |= bits #set comparator config bits
        self._write_register(CTRL_REG, tmp)
    
    def comparator_mode(self, bits):
        tmp = self._read_register(CTRL_REG)
        tmp &= ~(COMP_MODE_MASK | OS_MASK) #clear comparator config bits
        tmp |= (bits << 4) #set comparator config bits
        self._write_register(CTRL_REG, tmp)
  
    def comparator_thres(self, lo, hi):
        #get PGA configuration
        pga_conf = self._read_register(CTRL_REG)
        pga_conf &= PGA_MASK #leave only PGA bits
        pga_conf = (pga_conf >> 9) #shift them to the left
        #check if inputs are in range of PGA setting
        lo = self._check_theshold_overflow(lo, pga_conf)
        hi = self._check_theshold_overflow(hi, pga_conf)
        #convert volts to tenths of microvolts it will enable to easily
        #(no lenghty float division) convert float to intenal voltage
        #representation of ADS1115
        lo *= 10000000000.0 
        hi *= 10000000000.0
        #check how many x nV steps fit in lo and hi thresholds
        if pga_conf == FSR_6V:
            lo = int(int(lo) / 1875000)
            hi = int(int(hi) / 1875000)
        elif pga_conf == FSR_4V:
            lo = int(int(lo) / 1250000)
            hi = int(int(hi) / 1250000)
        elif pga_conf == FSR_2V:
            lo = int(int(lo) / 625000)
            hi = int(int(hi) / 625000)
        elif pga_conf == FSR_1V:
            lo = int(int(lo) / 312500)
            hi = int(int(hi) / 312500)
        elif pga_conf == FSR_0_5V:
            lo = int(int(lo) / 156250)
            hi = int(int(hi) / 156250)
        elif pga_conf == FSR_0_2V:
            lo = int(int(lo) / 78125)
            hi = int(int(hi) / 78125)
        else:
            print("error")
        #save data to threshold registers
        self._register_write(LO_THRES_REG, lo)
        self._register_write(HI_THRES_REG, hi)

    def alert_rdy_polarity(self, bits):
        tmp = self._read_register(CTRL_REG)
        tmp &= ~(ALRT_RDY_POL_MASK | OS_MASK) #clear alert ready polarity config bits
        tmp |= (bits << 3) #set alert ready polarity config bits
        self._write_register(CTRL_REG, tmp)

    def comparator_latch(self, bits):
        tmp = self._read_register(CTRL_REG)
        tmp &= ~(COMP_LAT_MASK | OS_MASK) #clear alert ready polarity config bits
        tmp |= (bits << 2) #set alert ready polarity config bits
        self._write_register(CTRL_REG, tmp)   