#  Ultrasonic Range Sensor

Author: Maxwell Bakalos

Date: 2023-02-24


### Summary
This program uses an ultrasonic range sensor. The ouput of the sensor is read as a voltage into the ADC pin of the ESP32. Then this is converted into a distance in meters using the ratio: (Vcc/1024)/5mm, or (3.3v/1024)/5mm -~-> (Vcc(1000/1024) / 0.005) mV/meter. The reciprocal is used to convert from mili-volts to meters. This device is not very accurate at least when I have tested it...

### Sketches/Diagrams

![Ultrasonic Range Sensor (small)](https://user-images.githubusercontent.com/114166327/221327954-83e3da0c-725f-4ec8-90e5-7daf515cf430.JPG)
![Q2-14 conversion factor derivation](https://user-images.githubusercontent.com/114166327/221327886-34b7ea27-35d4-4507-9aad-8bd4abf53da8.JPG)
![Q2-14 terminal](https://user-images.githubusercontent.com/114166327/221327892-46528bfc-9669-4b07-be85-c67aa312d5f9.JPG)


### Modules, Tools, Source Used Including Attribution
The "(Vcc/1024)/5mm" formula is from the slides & part datasheet.

### Supporting Artifacts

