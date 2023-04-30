# Space and Occupancy

Authors: Madeline Hain, Maxwell Bakalos, Miguel Ianus-Valdiva, Emre Karabay

Date: 2023-04-29

### Summary
Each sensor is read by the ESP32 on a different pin, and then the data is formatted into a CSV print statement. This printed data is read on the port by the NodeJS server. The server then takes the new sample of the data and appends it to the end of a CSV file called "Serial_Output1.csv". The server then takes the entire csv file, reads it into an array, and then send it to the HTML webpage. The HTML code reads each column of the array (each sensor's data" and plots them using multiple graphs in CanvasJS

### Self-Assessment 

| Objective Criterion | Rating | Max Value  | 
|---------------------------------------------|:-----------:|:---------:|
| Objective One | 1 |  1     | 
| Objective Two | 1 |  1     | 
| Objective Three | 1 |  1     | 
| Objective Four | 1 |  1     | 
| Objective Five | 1 |  1     | 
| Objective Six | 1 |  1     | 
| Objective Seven | 0.5 |  1     | 


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
- [Link to video demo](). Not to exceed 120s


### Modules, Tools, Source Used Including Attribution

### References


