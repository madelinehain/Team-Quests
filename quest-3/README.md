# Cat Tracker
Authors: Maxwell Bakalos, Miguel Ianus-Valdiva, Emre Karabay, Madeline Hain

Date: 2023-03-31

### Summary
Our device uses a NodeJS server running on a laptop which communicates between two ESP32s and the html webpage, via the router. One ESP32 has a thermistor and sends the data every 2 seconds over to the server, the other ESP32 has an LED and will blink when it receives a message from the server. 

The server receives the thermistor data via wifi through the router at port 3333, stores it in a CSV file called "Sensor_Output.csv", reads the csv data into an array, and passes that array onto the html page for display in a CanvasJS graph.

The html page has a button which toggles the LED to blink. WHen the button is pressed, the node server is notified and it sends a message to the LED ESP32 to either start or stop blinking.

The raspberry pi is set up to stream video (ssh into the Rpi, then use the command "sudo motion" to start streaming). This video is sent to port 8081 and then displayed in the html page.

### Self-Assessment 

| Objective Criterion | Rating | Max Value  | 
|---------------------------------------------|:-----------:|:---------:|
| Objective One | 1 |  1     | 
| Objective Two | 1 |  1     | 
| Objective Three | 0.5 |  1     | 
| Objective Four | 1 |  1     | 
| Objective Five | 1 |  1     | 
| Objective Six | 1 |  1     | 
| Objective Seven | 1 |  1     | 


### Solution Design

![Q3 Top of ObservationPage](https://user-images.githubusercontent.com/114166327/229256040-9d7e125b-1543-4c3e-a0dc-282966775184.jpg)
<p align="center">
Top of Observation Page
</p>
<br>


![Q3 Bottom of ObservationPage](https://user-images.githubusercontent.com/114166327/229256046-b59049e3-7b13-44c2-ba97-7a387c12e816.jpg)
<p align="center">
Bottom of Observation Page
</p>
<br>

![Q3 Node Server Terminal](https://user-images.githubusercontent.com/114166327/229256343-e6f6925f-7447-43d6-9a66-dab79fa48305.JPG)

Node Server Terminal
<br>

![Q3 THermistor CSV output](https://user-images.githubusercontent.com/114166327/229256553-0360a6ce-5c13-4e34-8694-7cdd1182079a.JPG)

<b> . . . </b>

![Q3 THermistor CSV output 2](https://user-images.githubusercontent.com/114166327/229256803-4ddb5895-6f0c-4079-96a3-629092d37835.JPG)


Thermistor CSV Output

### Sketches/Diagrams

![Diagram1](https://user-images.githubusercontent.com/114166327/230671529-d88561ef-b610-4867-8b27-a7e09425a16a.JPG)

<p align="center">
Client-Server Diagram
</p>

![Diagram2](https://user-images.githubusercontent.com/114166327/230671641-a0544f15-793a-4c5d-b8f5-3957e484f610.JPG)

<p align="center">
Server Diagram
</p>


### Supporting Artifacts
- Technical Presentation: https://drive.google.com/file/d/1Y7Kx-8gxULcwVduJsPyMwWg7OotmvK7r/view?usp=share_link
- Video Demo: https://drive.google.com/file/d/19QsmcVMaM6SSxlMqfbJiu507Hwc7yrzf/view?usp=share_link


### Modules, Tools, Source Used Including Attribution
Express for communication between html and node server (https://expressjs.com/)
### References


