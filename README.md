# Finite State Machine Implementation of Marvin 18
This is an offseason project structured around the development of a Finite State Machine (FSM) architecture to manage complex robot behaviors in a structured and reliable way. Its intent is to use Marvin 18, a relatively simple robot, as a testbed for creating an FSM and improve our understanding of the model before the next FRC season.

This effort is intended to improve the robot’s autonomy, response time, and task execution consistency by organizing actions into well-defined states and transitions. By clearly separating robot logic into manageable states (such as intake, alignment, scoring, and idle), the FSM allows for more maintainable and testable code while reducing the chances of unexpected behavior or “getting stuck” between commands. It intends to minimize delay and maximize computational efficiency of methods executed on the robot.

This project is part of a broader goal to bring more robust software engineering practices into our robot codebase. As we iterate and test this FSM model on Marvin 18, more features will be added.
## Current Control Scheme (v1, updated July 21st)

<img src="https://github.com/MARSProgramming/fsm-implementation-m18/blob/main/marvinFSM/ModeCoral.png" alt="controller_scheme_algae" width="800">
<img src="https://github.com/MARSProgramming/fsm-implementation-m18/blob/main/marvinFSM/ModeAlgae.png" alt="controller_scheme_coral" width="800">
<img src="https://github.com/MARSProgramming/fsm-implementation-m18/blob/main/marvinFSM/ModeNoPiece.png" alt="controller_scheme_nopiece" width="800">

## Reef Side Labels for Auto (from 2910)
<img src="https://github.com/MARSProgramming/fsm-implementation-m18/blob/main/marvinFSM/AutonomousReefLabelling.png" alt="reef_map" width="800">

# Project Tracker - Completions, Todos, & Future Plans 
- 7/21: So far, a lot has been accomplished. The core structure of code has been created. I estimate approx. 80% completion of this codebase.

## Completed
- [X] Create project and install dependencies
- [X] Develop IO architecture for all subsystems
- [X] Implement state machine logic for subsystems
- [X] Refactor 2910's 2025 REEFSCAPE vision code for PhotonVision
- [X] Determine and create superstructure ENUMs for state management
- [X] Create superstructure layout
- [X] Complete superstructure
- [X] Determine how to create a single-operator control scheme with toggled input (implemented, v0)
    - On this topic, we can add more states that allow for manual elevator setpoints for a vision failsafe. We have enough buttons to do so.
    - Implement v1
- [X] Add timeout mechanisms to elevator zeroing, shooting on each level, and spitting algae.
- [X] Because we don't have a sensor to detect if we have algae, we can isntead make this a driver input. (Ex. pressing left stick enters Algae Mode, and this sets the state commands for the entire controller to Algae.)
- [X] Implement Choreo and autonomous functionality (Autonomous factories and file handling for Choreo)
    - understand that with Drive to point autos, we don't even need Choreo. We can just PID to point for every single action. Our robot layout allows us to do this comfortably. 
- [X] Populate RobotContainer with subsystems and superstructure
- [X] Configure LoggedRobot structure 

## In-Progress 
- [ ] Implement testing controls
- [ ] Implement v1 control scheme in RobotContainer
- [ ] Implement all auto plays 
- [ ] Clean up code for review
    - Check for "stuck" states - if we enter a state, will the robot get stuck there despite callbacks.
    - Remove unecessary methods
    - Make code readable

- *Considering*: Add Superstructure states that command the elevator to a setpoint, independent of autoalign. Failsafe for vision.
- *Considering*: implement L1 scoring
- *Considering*: Create an Elastic layout concept.

## Planned
- [ ] In-depth, comprehensive analysis of superstructure, susbystems, and how the state machine is implemented to ensure a reliable robot for testing
- [ ] Real-world testing of system and PID tuning 
- [ ] Tuning of autonomous functionality
- [ ] LED functional tuning (not a priority)
- [ ] Add failure mode management - if a sensor fails, the robot should still be able to perform tasks.
- [ ] Add support for Limelight 4s if we decide to shift to them for next season.

## Goals
1. During our season, our best performance was a 3-piece autonomous, one of the pieces being on L2. This project aims to push Marvin 18 to a 3 piece, L4 autonomous play, with a high target of a 3 piece, 1 preload autonomous play.
2. Our alignment implementation was reliable by the end of the season, but we are looking for ways to improve its functionality. Utilizing 2910's Swerve code this year as a base, we strive for higher effectiveness and minimized alignment time with this model.
3. We want to improve driver experience by creating a framework to support a single-operator ideology, which is far more efficient than two operators (at least for Reefscape).
4. Improve LED state handling and create more robust visual states.
5. Optimize development times and give our team a better understanding of what to prioritize during a season when developing software for a robot.


