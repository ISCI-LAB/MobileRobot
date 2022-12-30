import wiringpi
import time

wiringpi.wiringPiSetup()
wiringpi.pinMode(7, 0)

while(1):
	output = wiringpi.digitalRead(7)
	print(output)
	time.sleep(0.5)
