# <div align="center">Robojeep</div>
![image](https://github.com/WinstonHChou/winter-2024-final-project-team-7/assets/68310078/0ba1c6cb-c9e0-4cf7-905a-f5f16e6bb2ca)
### <div align="center"> MAE 148 Final Project </div>
#### <div align="center"> Team 1 Fall 2025 </div>

<div align="center">
    <img src="images\ucsdrobocar-148-07.webp" width="800" height="600">
</div>

## Table of Contents
  <ol>
    <li><a href="#team-members">Team Members</a></li>
    <li><a href="#abstract">Abstract</a></li>
    <li><a href="#what-we-promised">What We Promised</a></li>
    <li><a href="#accomplishments">Accomplishments</a></li>
    <li><a href="#challenges">Challenges</a></li>
    <li><a href="#final-project-videos">Final Project Videos</a></li>
    <li><a href="#hardware">Hardware</a></li>
      <ul>
            <li><a href="#electronics">Electronics</a></li>
            <li><a href="#wiring">Wiring</a></li>
        </ul>
    <li><a href="#software">Software</a></li>
        <ul>
            <li><a href="#slam-simultaneous-localization-and-mapping">SLAM (Simultaneous Localization and Mapping)</a></li>
            <li><a href="#obstacle-avoidance">Obstacle Avoidance</a></li>
        </ul>
    <li><a href="#gantt-chart">Gantt Chart</a></li>
    <li><a href="#course-deliverables">Course Deliverables</a></li>
    <li><a href="#project-reproduction">Project Reproduction</a></li>
    <li><a href="#acknowledgements">Acknowledgements</a></li>
    <li><a href="#Ideas for future Teams">Ideas for future Teams</a></li>
  </ol>

<hr>

## Team Members
Anderson Compalas - ECE

Jobanpreet Mutti  - MAE

Saul Armenta - ECE

Bastian Müllner - MAE - UPS student visiting from TUM
<hr>

## Abstract
The projects goal is to make a kids toy car controllabe via Ros2, to provide a bigger robot car platform for future final projects. To archieve this, all of the stock car electronics besides the motors and batterys are replaced with robot componets.

<hr>

## What We Promised
### Must Have
* Fully functional low level robot control

### Nice to Have
* Artificial differential
* Skid steering
* Traction control

<hr>

## Accomplishments
* Robot is controllable via Ros2
  * Steering angle with closed loop control
  * Skid Steering
  * Forward and backward driving
* Complete Hardware installation for encoders on all 4 wheel motors
* Safe operation with seperate estop relay controlled dircly via the remote control bypassing the jetson nano
<hr>

## Challenges
* Passing the encoder feedback to the jetson to publish in Ros2
* Time
* Hardware issues. The gears inside the steering motor assembly were stripped, but luckily the bed lifting motor had the same gears to replace them
<hr>

## Hardware

> [!WARNING]
> Charge the battery very regularly, there is no undervoltage protection!
> It should always stay above 22.5V. If you want to check it, pull out one of the power cables in the 24V splitter in the front and use a multimeter

> [!IMPORTANT]
> The Emergency stop is the Top right button on the remote, when the green light is on it is triggered. All H-Drives need a 5V enable signal to make the motors move at all. If nothing moves, probably the estop is trigerred disabling all of the 5V enable circuit. If the red led on the relay board is on it is triggered. If parts of the car wont move, probably the enable wire is broken or unplugged somewhere. All the enable wires are yellow.

### Electronics:
* Nvidia Jetson Nano: Main controller
* 2 Arduino Mega 2560:
  * Controlling all the H-Bridges
  * Reading feedback from 2 of the wheel encoders
  * Reading the steering encoder
* 2 Arduino micro pro: Reading the other two wheel encoders. Since the used encoders dont support changing their i2c adress, we can only put one per i2c bus, so one per arduino
* 5 H-Bridges: Controlling the brushed motors (https://www.amazon.com/MTDELE-H-Bridge-semiconductor-Refrigeration-Controller/dp/B0D732VYGZ/ref=sims_dp_d_dex_popular_subs_t3_v6_d_sccl_1_4/136-0486074-4523801?psc=1)
* 4 AS5600 Hall effect encoders: Reading the wheel motor rpm
* REV-11-1271 Absolute encoder: Reading the steering angle
* Radio control:
  * Radiomaster reciever: recieving commands from remote control
  * Arduino micro pro: Converting it to a usb joystick for the jetson to read
  * Relay board: When triggert it stops 5V enable signal to ALL H-Bridges, so nothing will move if this 5v signal is not switched.

 ### Wiring
<div align="left">
    <img src="images\Robojeep wiring.png" width="800" height="600">
</div>

<hr>

### Car Hardware
We improved the steering play by printing a spacer to take up the void created by the manufacturing tolerances, this greatly improved our steering accuracy
<div align="left">
    <img src="images\Robojeep wiring.png" width="400" height="300">
</div>

If this car should be used to transport a large payload, the seats can be removed 

## Ideas for future Teams
* Add a Battery sensor: The battery shouldnt drop below 22.5V and currently there is nothing warning you or stopping the car
* Artificial differential: Compute the wheel angle and use that to compute how much faster the outer wheels in a turn need to spin
* Wire all the existing car dash and steering wheel switches to the front arduino to make them usable in your code

<hr>
