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



### Sketches/Diagrams
<p align="center">
<img src="./images/ece444.png" width="50%">
</p>
<p align="center">
Caption Here
</p>



### Supporting Artifacts
- [Link to video technical presentation](). Not to exceed 120s
- Video Demo: https://drive.google.com/file/d/19QsmcVMaM6SSxlMqfbJiu507Hwc7yrzf/view?usp=share_link


### Modules, Tools, Source Used Including Attribution
Express for communication between html and node server (https://expressjs.com/)
### References


