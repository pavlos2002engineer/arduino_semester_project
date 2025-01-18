# arduino_semester_project
Circuit with thermistor, relay and step-motor

# thermistor
Circuit that implements the function of a temperature sensor, with a thermistor (NTC). Specifically, we had to design the circuit to read the temperature from the sensor and display the temperature to the LCD. Then, activate a LED when the value exceeds a certain limit either upwards (RED) or downwards (BLUE). 

# relay
The circuit that operates a simple relay and at the same time checks its correct operation. If there is a fault in the relay, a red indicator light (LED) should light up. The falut might be broken gorund of the relay, power supply cut to relay or LED and the normally open switch cut.

# step motor
A circuit that reads voltage values of 0.5 - 4.5V from the potensiometer (10kΩ) and generate a corresponding PWM signal. We should activate with this signal a valve or a motor. We chose to activate a step motor, that will rotate 512 half - steps in one direction, delay for 4 sec and rotate again 512 steps for the opposite direction. 
