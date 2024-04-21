# pysunray test
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
sys.path.append("./build/")
import time
import pysunray as sun

print('--------------start------------')
time.sleep(2.0)

#ble = sun.BleUartServer()
ble = sun._SerialBLE
ble.begin(115200)

mot = sun._motor
driver = sun._robotDriver
driver.begin()

while True:
    if ble.available():
        s = ''
        while ble.available():
            s +=chr(ble.read())
        print(s)

    #print('.')
    time.sleep(0.1)

    mot.setLinearAngularSpeed(0, 0, True)
    driver.run()

    


