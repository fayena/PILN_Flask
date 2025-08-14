import time
import board
import busio
import digitalio
import adafruit_max31856
     
# create a spi object
spi = busio.SPI(board.SCK, board.MOSI, board.MISO)
     
# allocate a CS pin and set the direction
cs = digitalio.DigitalInOut(board.D5)
cs.direction = digitalio.Direction.OUTPUT
     
# create a thermocouple object with the above
thermocouple = adafruit_max31856.MAX31856(spi, cs)
     
# print the temperature!
while 1:
    print(thermocouple.fault)
    faults = thermocouple.fault
    for x in faults.values():
        if x == False:
            print ("True")
        else:
            print ("False")
    print(thermocouple.temperature)
    time.sleep(1)	 	
