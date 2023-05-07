#  Buggy

Author: Maxwell Bakalos, Miguel Ianus-Valdiva, Emre Karabay, Madeline Hain

Date: 2023-04-24


### Summary
This skill covers the basis of basic buggy controls, allowing us to set a steering angle from -90 to 90 on the servo and a throttle from -100 to 100 (as a percentage of full throttle, 0 being stationary). This is done via user input in the console, requiring the buggy to be wired into a PC at all times. 

### Sketches/Diagrams
N/A

### Modules, Tools, Source Used Including Attribution
- Console I/O
  - UART communication between ESP and PC for user input
- Tasks
  - Use of tasks allows us to continually read the serial monitor, and update the motor speed and steering angle as necessary
- PWM management
  - Both the motor driver and the servo rely on PWM inputs to actuate

### Supporting Artifacts
- [Video Link](https://drive.google.com/file/d/1wZhnyOjNwTti8atJyUHahcCoCvTtUQBm/view?usp=sharing)

