# Finite State Machine Implementation of Marvin 18
This is an offseason project structured around the development of a Finite State Machine (FSM) architecture to manage complex robot behaviors in a structured and reliable way. Its intent is to use Marvin 18, a relatively simple robot, as a testbed for creating an FSM and improve our understanding of the model before the next FRC season.

This effort is intended to improve the robot’s autonomy, response time, and task execution consistency by organizing actions into well-defined states and transitions. By clearly separating robot logic into manageable states (such as intake, alignment, scoring, and idle), the FSM allows for more maintainable and testable code while reducing the chances of unexpected behavior or “getting stuck” between commands. It intends to minimize delay and maximize computational efficiency of methods executed on the robot.

This project is part of a broader goal to bring more robust software engineering practices into our robot codebase. As we iterate and test this FSM model on Marvin 18, more features will be added.

## Project Progress
- [X] Create project and install dependencies
- [X] Develop IO architecture for all subsystems
- [X] Implement state machine logic for subsystems
- [X] Refactor 2910's 2025 REEFSCAPE vision code for PhotonVision
- [X] Determine and create superstructure ENUMs for state management
- [X] Create superstructure layout
- [X] Complete superstructure
- [X] Determine how to create a single-operator control scheme with toggled input (implemented, v1)
    - On this topic, we can add more states that allow for manual elevator setpoints for a vision failsafe. We have enough buttons to do so.
- [ ] Implement testing controls and functionality
- [X] Add timeout mechanisms to elevator zeroing, shooting on each level, and spitting algae.
- [X] Because we don't have a sensor to detect if we have algae, we can isntead make this a driver input. (Ex. pressing left stick enters Algae Mode, and this sets the state commands for the entire controller to Algae.)
- [ ] Implement Choreo and autonomous functionality (Autonomous factories and file handling for Choreo)
- [X] Populate RobotContainer with subsystems and superstructure
- [ ] Configure LoggedRobot structure (In progress)
- [ ] Clean up code for review

## Planned
- [ ] In-depth, comprehensive analysis of superstructure, susbystems, and how the state machine is implemented to ensure a reliable robot for testing
- [ ] Real-world testing of system and PID tuning 
- [ ] Tuning of autonomous functionality
- [ ] Add failure mode management - if a sensor fails, the robot should still be able to perform tasks.
- [ ] Add support for Limelight 4s if we decide to shift to them for next season.

## Goals
1. During our season, our best performance was a 3-piece autonomous, one of the pieces being on L2. This project aims to push Marvin 18 to a 3 piece, L4 autonomous play, with a high target of a 3 piece, 1 preload autonomous play.
2. Our alignment implementation was reliable by the end of the season, but we are looking for ways to improve its functionality. Utilizing 2910's Swerve code this year as a base, we strive for higher effectiveness and minimized alignment time with this model.
3. We want to improve driver experience by creating a framework to support a single-operator ideology, which is far more efficient than two operators (at least for Reefscape).
4. Improve LED state handling and create more robust visual states.
5. Optimize development times and give our team a better understanding of what to prioritize during a season when developing software for a robot.


