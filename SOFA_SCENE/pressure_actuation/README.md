# Pressure Actuation

Contains Python and Arduino script to connect and use the pneumatic actuation with a MegaPi

# to test serial connexion :
Put the arduino script "test_serie.ino" and launch the python script "test_serial.py" with the command "python test_serial.py" in the terminal.
You should have the pressure you choosed as answer in the terminal, for example : "[b'Good got: 0.20,0.20,0.20\r\n']"

# to set a pressure from a python script:
Put the arduino script "Pressure_Actuation.ino" and launch the python script "set_pressure.py" with the command "python set_pressure.py" in the terminal.
Be careful, and check if the pressure that is setted in the python script is the pressure you want to apply.
Also, be careful, there is a maximum pressure security value in the Arduino script, that is setted to 1.5 bar. This is a quite high maximum value (but needed for my application). It would be more prudent to put 0.5bar and then 1bar maximum pressure for your 1st tests, just to be sure there is no problem.
The python script is sending always the same frame with while(1) loop, if you want to stop to apply this pressure, just made a CTRL-C in the terminal.