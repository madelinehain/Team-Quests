#  UDP

Author: Maxwell Bakalos, Miguel Ianus-Valdiva, Emre Karabay, Madeline Hain

Date: 2023-03-24


### Summary
#### Part 1: Laptop is Server, ESP32 is Client <br>
In the ESP32 menuconfig we changed the IPV4 address to the laptop's IP address and in the node server code we used the same IP address. Port 3333 was used. We also had to set up the router in menuconfig and used our SSID, "Group_5", and password, "smartsys".

#### Part 2: Laptop is Client, ESP32 is Server <br>
In the ESP32 menuconfig we set up the router in menuconfig and used our SSID, "Group_5", and password, "smartsys". The laptop ran a Node client which sent "Hello ESP32!" to the server 5 times.

### Sketches/Diagrams
#### Part 1: Laptop is Server, ESP32 is Client
https://user-images.githubusercontent.com/114166327/227614331-acae2353-9196-4e90-b373-f4c79da03bd2.MOV

#### Part 2: Laptop is Client, ESP32 is Server
![Q3-24__UDP_2](https://user-images.githubusercontent.com/114166327/227623381-af0e98ac-e67e-4456-b541-6ed5be328a33.JPG)


### Modules, Tools, Source Used Including Attribution
Part 1: https://github.com/espressif/esp-idf/tree/master/examples/protocols/sockets/udp_client <br>
Part 2: https://github.com/espressif/esp-idf/tree/master/examples/protocols/sockets/udp_server

### Supporting Artifacts

