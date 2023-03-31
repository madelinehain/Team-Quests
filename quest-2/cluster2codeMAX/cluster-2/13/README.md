#  Thermistor (Temperature Sensor)

Author: Maxwell Bakalos

Date: 2023-02-23


### Summary
I used the thermistor probe in a voltage divider with a 10KOhm resistor. The voltage read from this voltage divider was used with the voltage divider equation to find the resistance in the thermistor. Then the resistance was mapped to a corresponding temperature based on the thermistor datasheet. The measured resistance value was compared to its upper & lower bounds in the resistance table, then the corresponding upper & lower bounds in the temperature table were interpolated based on the position of the resistance.

### Sketches/Diagrams

![InkedQ2-13 Thermistor Voltage Divider (small) (draw)](https://user-images.githubusercontent.com/114166327/221061479-e84c678f-b1ef-433f-b79d-c8ef0857d776.jpg)
![Q2-13 Voltage Divider Diagram (small)](https://user-images.githubusercontent.com/114166327/221061005-8c4e305d-e32f-4b0f-ad69-c8187e93ad46.jpeg)

<p align="center">
Thermistor Voltage Divider & Equations
</p>

<br>


![Q2-13_Thermistor (small)](https://user-images.githubusercontent.com/114166327/221061006-ce396ab9-1640-4a6c-a36c-ab9893dda848.jpeg)


<p align="center">
Thermistor going from Room Temp -> Hand Temp
</p>

![Q2-13 terminal (small)](https://user-images.githubusercontent.com/114166327/221061003-c67c92a6-d99d-445f-9c71-d4efb3779c2e.JPG)

<p align="center">
Terminal View of Thermistor going from Room Temp -> Hand Temp
</p>

### Modules, Tools, Source Used Including Attribution
- [Thermistor Datasheet](https://www.eaa.net.au/PDF/Hitech/MF52type.pdf)

### Supporting Artifacts

