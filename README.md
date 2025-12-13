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
    <li><a href="#software">Software</a></li>
        <ul>
            <li><a href="#slam-simultaneous-localization-and-mapping">SLAM (Simultaneous Localization and Mapping)</a></li>
            <li><a href="#obstacle-avoidance">Obstacle Avoidance</a></li>
        </ul>
    <li><a href="#hardware">Hardware</a></li>
    <li><a href="#gantt-chart">Gantt Chart</a></li>
    <li><a href="#course-deliverables">Course Deliverables</a></li>
    <li><a href="#project-reproduction">Project Reproduction</a></li>
    <li><a href="#acknowledgements">Acknowledgements</a></li>
    <li><a href="#contacts">Contacts</a></li>
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
