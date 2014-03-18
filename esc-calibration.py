
from motor import motor

motor1 = motor('m1', 17, simulation=False)
motor2 = motor('m1', 18, simulation=False)
motor3 = motor('m1', 25, simulation=False)
motor4 = motor('m1', 22, simulation=False)

motors = [motor1, motor2, motor3, motor4]

print('***Disconnect ESC power')
print('***then press ENTER')
res = raw_input()
try:
        for mitour in motors:
                mitour.start()
                mitour.setW(100)

	print('***Connect ESC Power')
	print('***Wait beep-beep')

        res = raw_input()
	for mitour in motors:
                mitour.start()
                mitour.setW(0)
	print('***Wait N beep for battery cell')
	print('***Wait beeeeeep for ready')
	print('***then press ENTER')
	res = raw_input()
	
	for mitour in motors:
                mitour.start()
                mitour.setW(10)
	res = raw_input()
finally:
    # shut down cleanly
        for mitour in motors:
                mitour.stop()

        print ("well done!")
        exit()
