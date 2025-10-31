# FrED-TEC
These are the codes for the controllers and GUI of the FrED designed on TEC MTY.

To run it follow the next steps:
1. Clone the repository or download the zip file with all the files.
2. Copy and paste the Arduino files in ArduinoIDE: MAIN.ino, MOTORDC.h, HOTEND.h, PIN_MAP.h in the same project.
3. Plug in the Arduino Mega and upload the MAIN.ino with its respective C# files. After verying the files were uploaded, close ArduinoIDE.
4. Plug in the power supply to the RAMPS 1.4 (The shield on the Arduino Mega).
5. Open the file mainGUI.py in Visual Studio Code and run it while connected to the Arduino Mega.

Debugging tips:
1. If the mainGUI.py file freezes or does not run, check manually the port at which your laptop is connected to the arduino and change it on the section of "Manual Ports".
