# Quest Name

Authors: Maxwell Bakalos, Miguel Ianus-Valdiva, Madeline Hain, Emre Karabay

Date: 2023-04-23

<br>

## Summary
Our self-driving buggy uses one LiDAR facing the righthand wall and another LiDAR facing the front wall. This design allows the buggy to maintain a certain distance from the right wall and also turn left to avoid hitting a front wall. 

#### Wheel Motor Speed
The wheel speed is set with a PID controller (task). First it reads in the wheel speed with the printed out striped disk and the pulse counter (task), converts the pulses per 0.5 seconds to speed in m/s. Then the PID values are calculated based on the setpoint of 0.35m/s and the motor speed is sent to the motor controller (task) which sets the PWM values for the wheel motor.

#### Right LiDAR:
The LiDARs are read periodically into a PID which controls the wheel angle servo. It wants to maintain its setpoint of 40cm and turns the wheels left and right to adjust. There is a single task which reads from both LiDARs (task). 

#### Front LiDAR:
When the front distance to an obstacle is measured to be under 180cm it enters "left turn mode" where the Right LiDAR PID is overrulled and the wheels turn a hard left. If the front distance is measured to be under 80cm then the buggy automatically stops moving until there is no obstacle. 

#### Wireless Start/Stop
A Node JS server runs on a host laptop. An HTML page has buttons "START", "STOP", and "EMERGENCY STOP" which, when clicked, send messages to the buggy ESP32 ("START", "STOP", "EMERGENCY_STOP"). When the buggy turns on (or the server turns on for the first time) the message is "Ok!" which results in the same response as "STOP": the buggy's wheel motors are set to speed zero, so it won't start until the user clicks "START". The buggy connects to the "Group_5" WiFi router when it starts up and receives the server messages (task).

Once "START" is clicked, the HTML page uses jQuery to send each of the button responses from either "/startButton", "/stopButton", or "/emergencyStopButton". In the "Server_n_Webpage" folder is the "jQuery_File" folder which contains a local copy of jQuery instead of web link because sometimes the BU ethernet ports don't give internet access to the router.

Then the server receives one of these messages and sends out either "START", "STOP", or "EMERGENCY_STOP" to the ESP32 via WiFi. "START" makes the buggy move forward at its setpoint speed of 0.35m/s. "STOP" makes the speed zero. "EMERGENCY_STOP" makes the speed zero and resets the buggy. The buggy can switch between "START and "STOP" modes indefinitly, but if "EMERGENCY_STOP" is pressed, it will have to reset first.

#### Alphanumeric Display
Every 10ms the Alphanumeric display (task) is updated with the wheel speed read fro mthe pulse counter.
<br></br>

## Self-Assessment 

| Objective Criterion | Rating | Max Value  | 
|---------------------------------------------|:-----------:|:---------:|
| Objective One | 1 |  1     | 
| Objective Two | 1 |  1     | 
| Objective Three | 1 |  1     | 
| Objective Four | 1 |  1     | 
| Objective Five | 1 |  1     | 
| Objective Six | 1 |  1     | 
| Objective Seven | 1 |  1     | 


## Solution Design



## Sketches/Diagrams
<p align="center">
<img src="./images/ece444.png" width="50%">
</p>
<p align="center">
Caption Here
</p>



## Supporting Artifacts
- [Link to video technical presentation](). Not to exceed 120s
- [Link to video demo](). Not to exceed 120s


## Modules, Tools, Source Used Including Attribution

## References


