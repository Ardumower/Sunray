# pysunray test  (steer robot via Dabble App)
#
#  install pybind11 with:  pip install "pybind11[global]"
#
#  cmake .. 
#  cmake -DCMAKE_C_COMPILER=/usr/bin/gcc -DCMAKE_CXX_COMPILER=/usr/bin/g++ ..
#
# to run the python interpreter as sudo:
#  1. which python  =>    /home/alex/miniconda3/envs/py38/bin/python
#  2. sudo /home/alex/miniconda3/envs/py38/bin/python test.py

import sys
import os
sys.path.append("./build/")
import time
import pysunray as sun

print('--------------start------------')
#time.sleep(2.0)

# Dabble App
dabble = sun._Dabble
gamepad = sun._GamePad

#ble = sun.BleUartServer()
ble = sun._SerialBLE
mot = sun._motor
driver = sun._robotDriver

#ble.begin(115200)
driver.begin()
dabble.begin('test')

toolOn = False
maxSpeed = 0.5

buttonTimeout = 0


while True:
    '''if ble.available():
        s = ''
        while ble.available():
            s +=chr(ble.read())
        print(s)'''
    
    dabble.processInput() 

    x = gamepad.getXaxisData()
    y = gamepad.getYaxisData()

    if time.time() > buttonTimeout:
        if gamepad.isUpPressed():
            print('isUpPressed')
        elif gamepad.isSquarePressed():
            os.system('shutdown now')
        elif gamepad.isCirclePressed():
            toolOn = not toolOn
            buttonTimeout = time.time() + 0.5
        elif gamepad.isCrossPressed():
            maxSpeed = 0.5
        elif gamepad.isTrianglePressed():
            maxSpeed = 1.0

    
    linearSpeed = y / 6.0 * 0.2
    angularSpeed = x / 6.0 * 0.1 

    print('x', round(x, 1), 'y', round(y, 1), toolOn)

    mot.setLinearAngularSpeed(linearSpeed, angularSpeed, True)
    mot.setMowState(toolOn);   
    mot.run()

    driver.run()

    #print('.')
    time.sleep(0.01)



