# cgmoi
HX711 load cell: 

Refer to https://makersportal.com/blog/2019/5/12/arduino-weighing-scale-with-load-cell-and-hx711  for instructions on how to connect load cell to HX711 to Arduino. All 3 load cells can share a common power and ground connection through a breadboard. 

Download to library at https://github.com/olkal/HX711_ADC for examples on calibrating and reading values from multiple load cells. 

To calibrate load cell, connect Arduino to an individual load cell through HX711 and run “Calibration” example. Follow instructions on the serial monitor and record down “calibration value”.  Do this for every individual load cell. 

Use HX711_script3.ino to read all 3 load cells. Input “calibration values” of each respective load cell into the code. Weight reading is taken 10 times with 0.05sec intervals and average weight on each load cell is obtained. Calculations are then made with the average weights. Run the code and read values of weight and coordinates of CG from serial monitor. 

Send “t” on serial monitor to tare load cells immediately. 

Reading is in grams. CG units are in mm. 
