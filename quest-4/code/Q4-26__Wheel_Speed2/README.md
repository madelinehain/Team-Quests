#  Wheel Speed

Author: Maxwell Bakalos, Miguel Ianus-Valdiva, Emre Karabay, Madeline Hain

Date: 2023-04-24


### Summary
In this skill, we implement a pulse counter to measure the wheel speed and then deduce the overall speed of the buggy. By placing an encoder pattern on the wheel, we can measure how many times whe wheel has spun for a given time period by taking advantage of the different reflective properties of black and white surfaces using an IR emmitter and receiver. This brings us one step closer to being able to implement a PID controller for the buggy speed, allowing us to accelerate and decelerate smoothly, as well as maintain a constant speed in varying conditions (ie. different surface frictions). 

### Sketches/Diagrams
N/A

### Modules, Tools, Source Used Including Attribution
- Pulse counting tools in Espressif
- ADC to read the IR receiver

### Supporting Artifacts
- [Video Link](https://drive.google.com/file/d/1rZdXnt99JzBGb7A7zf9eALdlI-8nZz4o/view?usp=sharing)
