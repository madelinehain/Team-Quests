# NFC Scooter Key Fob

Authors: Maxwell Bakalos, Miguel Ianus-Valdiva, Madeline Hain, Emre Karabay, 

Date: 2023-04-30

### Summary
We created an IR scooter locking system using two ESP32s, a raspberry pi with a camera, and a node js server database. The keyfob QR code reader reads a QR code and sends the message over wifi to the node server database. The database validates the {scooter ID, fob ID} pair then sends out a random key to the two ESP32 via wifi. The keyfob ESP32 then uses an IR signal to send the random key over to the scooter ESP32 which will unlock it if correct.

### Self-Assessment 

| Objective Criterion | Rating | Max Value  | 
|---------------------------------------------|:-----------:|:---------:|
| Objective One | 1 |  1     | 
| Objective Two | 1 |  1     | 
| Objective Three | 1 |  1     | 
| Objective Four | 1 |  1     | 
| Objective Five | 1 |  1     | 
| Objective Six | 1 |  1     | 
| Objective Seven | 1 |  1     | 


### Solution Design
The LED on the scooter will be red if the random key has not been sent, blue if it has been sent, and green if the key was sent and the IR signal from the keyfob matches the random key.

<p align="center">
<img src="./images/Q5 stuff.JPG" width="50%">
</p>
<p align="center">
Diagram
</p>


### Sketches/Diagrams
<p align="center">
<img src="./images/Q5_skwetch.jpg" width="50%">
</p>
<p align="center">
Diagram
</p>



### Supporting Artifacts
- Technical Presentation: https://drive.google.com/file/d/1E4ysBiPVl1RZ68GqTHzZlVy5O5e8m4F-/view?usp=share_link
- Demo: https://drive.google.com/file/d/1zr9RxNJIx6D43QDOQEzHVgFjb_rXzSbo/view?usp=share_link


### Modules, Tools, Source Used Including Attribution

### References


