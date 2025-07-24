package frc.robot.subsystems;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotState;
import frc.robot.constants.Constants;
import frc.robot.constants.Field;
import frc.robot.subsystems.algae.AlgaeSubsystem;
import frc.robot.subsystems.coral.CoralSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.util.Dash;

public class Superstructure extends SubsystemBase {
    private final SwerveSubsystem swerveSubsystem;
    private final CoralSubsystem coralSubsystem;
    private final AlgaeSubsystem algaeSubsystem;
    private final LEDSubsystem ledSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;

    boolean coralSpitFlag = false;
    boolean algaeModeActive = false;

   Constants.SuperstructureConstants.ScoringSide scoringSide = Constants.SuperstructureConstants.ScoringSide.LEFT;
   Constants.SuperstructureConstants.ScoringLevel scoringLevel = Constants.SuperstructureConstants.ScoringLevel.L1;


    private final Dash operatorDashboard;
    private final CommandXboxController controller = new CommandXboxController(0);


    private Constants.SuperstructureConstants.AutomationLevel automationLevel =
    Constants.SuperstructureConstants.AutomationLevel.AUTO_RELEASE;
private Constants.SuperstructureConstants.ReefSelectionMethod reefSelectionMethod =
    Constants.SuperstructureConstants.ReefSelectionMethod.ROTATION;


    public enum WantedSuperState {
        HOME,
        STOPPED,
        DEFAULT_STATE,
        FORCE_RELOCALIZE,
        SCORE_L1_MANUAL_ALIGN,
        SCORE_LEFT_L2,
        SCORE_LEFT_L3,
        SCORE_LEFT_L4,
        SCORE_RIGHT_L2,
        SCORE_RIGHT_L3,
        SCORE_RIGHT_L4,
        MANUAL_L4,
        MANUAL_L3,
        MANUAL_L2,
        MANUAL_L1,
        HOLD_ALGAE,
        INTAKE_ALGAE_FROM_REEF_TOP,
        INTAKE_ALGAE_FROM_REEF_BOT,
        INTAKE_ALGAE_FROM_REEF_GROUND,
        INTAKE_CORAL_FROM_STATION,
        MOVE_ALGAE_TO_PROCESSOR_POSITION,
        SCORE_ALGAE_IN_PROCESSOR,
        PREP_CLIMB,
        CLIMB
    }

    public enum CurrentSuperState {
        HOME,
        STOPPED,
        HOLDING_CORAL_TELEOP,
        NO_PIECE_AUTO,
        NO_PIECE_TELEOP,
        HOLDING_CORAL_AUTO,
        HOLDING_ALGAE,
        FORCE_RELOCALIZE,
        SCORE_TELEOP_L1_MANUAL_ALIGNMENT,
        SCORE_LEFT_TELEOP_L2,
        SCORE_LEFT_TELEOP_L3,
        SCORE_LEFT_TELEOP_L4,
        SCORE_RIGHT_TELEOP_L2,
        SCORE_RIGHT_TELEOP_L3,
        SCORE_RIGHT_TELEOP_L4,
        SCORE_LEFT_AUTO_L2,
        SCORE_LEFT_AUTO_L3,
        SCORE_LEFT_AUTO_L4,
        SCORE_RIGHT_AUTO_L2,
        SCORE_RIGHT_AUTO_L3,
        SCORE_RIGHT_AUTO_L4,
        MANUAL_L4,
        MANUAL_L3,
        MANUAL_L2,
        MANUAL_L1,
        INTAKE_ALGAE_FROM_REEF_TOP,
        INTAKE_ALGAE_FROM_REEF_BOT,
        INTAKE_ALGAE_FROM_REEF_GROUND,
        INTAKE_CORAL_FROM_STATION,
        MOVE_ALGAE_TO_PROCESSOR_POSITION,
        SCORE_ALGAE_IN_PROCESSOR,
        PREP_CLIMB,
        CLIMB
    }

    private WantedSuperState wantedSuperState = WantedSuperState.STOPPED;
    private CurrentSuperState currentSuperState = CurrentSuperState.STOPPED;
    private CurrentSuperState previousSuperState;

    private boolean hasDriveToPointSetPointBeenSet = false;

   // private NeutralModeValue pastSwitchValue = NeutralModeValue.Brake;

    private boolean hasPoseBeenResetPrematch = false;

   // private boolean allowExternalCommandToAccessLEDS = false;
 

    public Superstructure(SwerveSubsystem swerve, CoralSubsystem coral, AlgaeSubsystem algae, ElevatorSubsystem elev, LEDSubsystem led, Dash dash) {
        swerveSubsystem = swerve;
        coralSubsystem = coral;
        algaeSubsystem = algae;
        elevatorSubsystem = elev;
        ledSubsystem = led;
        operatorDashboard = dash;
    }

    @Override
    public void periodic() {
      Logger.recordOutput("ReefSelectionMethod", reefSelectionMethod);

      Logger.recordOutput("Superstructure/hasProfileBeenSet", hasDriveToPointSetPointBeenSet);

      automationLevel = operatorDashboard.getAutomationLevel();

      Logger.recordOutput("Superstructure/WantedSuperState", wantedSuperState);
      Logger.recordOutput("Superstructure/CurrentSuperState", currentSuperState);
      Logger.recordOutput("Superstructure/PreviousSuperState", previousSuperState);   
      
      currentSuperState = handleStateTransitions();
      applyStates();

      if (DriverStation.isDisabled()) {
         if (elevatorSubsystem.limitRead() && coralSubsystem.hasCoral()) {
            ledSubsystem.setWantedAction(hasPoseBeenResetPrematch ? LEDSubsystem.WantedState.DISPLAY_READY_FOR_MATCH : LEDSubsystem.WantedState.DISPLAY_ROBOT_IS_PHYSICALLY_READY_FOR_MATCH);
         } 
      } else if (DriverStation.isAutonomous() && DriverStation.isEnabled()) {
         ledSubsystem.setWantedAction(LEDSubsystem.WantedState.DISPLAY_OFF);
      }
   }

   private CurrentSuperState handleStateTransitions() {
      previousSuperState = currentSuperState;

      switch (wantedSuperState) {
         default: 
            currentSuperState = CurrentSuperState.STOPPED;
            break;
         case HOME:
            currentSuperState = CurrentSuperState.HOME;
            break;
         case FORCE_RELOCALIZE:
            currentSuperState = CurrentSuperState.FORCE_RELOCALIZE;
            break;
         case SCORE_L1_MANUAL_ALIGN:
            currentSuperState = CurrentSuperState.SCORE_TELEOP_L1_MANUAL_ALIGNMENT;
            break;
         case SCORE_LEFT_L2:
            currentSuperState = DriverStation.isAutonomous() ? CurrentSuperState.SCORE_LEFT_AUTO_L2 : CurrentSuperState.SCORE_LEFT_TELEOP_L2;
            break;
         case SCORE_LEFT_L3:
            currentSuperState = DriverStation.isAutonomous() ? CurrentSuperState.SCORE_LEFT_AUTO_L3 : CurrentSuperState.SCORE_LEFT_TELEOP_L3;
            break;
         case SCORE_LEFT_L4:
            currentSuperState = DriverStation.isAutonomous() ? CurrentSuperState.SCORE_LEFT_AUTO_L4 : CurrentSuperState.SCORE_LEFT_TELEOP_L4;
            break;
         case SCORE_RIGHT_L2:
            currentSuperState = DriverStation.isAutonomous() ? CurrentSuperState.SCORE_RIGHT_AUTO_L2 : CurrentSuperState.SCORE_RIGHT_TELEOP_L2;
            break;
         case SCORE_RIGHT_L3:
            currentSuperState = DriverStation.isAutonomous() ? CurrentSuperState.SCORE_RIGHT_AUTO_L3 : CurrentSuperState.SCORE_RIGHT_TELEOP_L3;
            break;
         case SCORE_RIGHT_L4:
            currentSuperState = DriverStation.isAutonomous() ? CurrentSuperState.SCORE_RIGHT_AUTO_L4 : CurrentSuperState.SCORE_RIGHT_TELEOP_L4;
            break;
         case MANUAL_L1:
            currentSuperState = CurrentSuperState.MANUAL_L1;
            break;
         case MANUAL_L2:
            currentSuperState = CurrentSuperState.MANUAL_L2;
            break;
         case MANUAL_L3:
            currentSuperState = CurrentSuperState.MANUAL_L3;
            break;
         case MANUAL_L4:
            currentSuperState = CurrentSuperState.MANUAL_L4;
            break;
         case INTAKE_ALGAE_FROM_REEF_TOP:
            currentSuperState = CurrentSuperState.INTAKE_ALGAE_FROM_REEF_TOP;
            break;
         case INTAKE_ALGAE_FROM_REEF_BOT:
            currentSuperState = CurrentSuperState.INTAKE_ALGAE_FROM_REEF_BOT;
            break;
         case INTAKE_ALGAE_FROM_REEF_GROUND:
            currentSuperState = CurrentSuperState.INTAKE_ALGAE_FROM_REEF_GROUND;
            break;
         case MOVE_ALGAE_TO_PROCESSOR_POSITION:
            currentSuperState = CurrentSuperState.MOVE_ALGAE_TO_PROCESSOR_POSITION;
            break;
         case HOLD_ALGAE:
            currentSuperState = CurrentSuperState.HOLDING_ALGAE;
            break;
         case INTAKE_CORAL_FROM_STATION:
            currentSuperState = CurrentSuperState.INTAKE_CORAL_FROM_STATION;
            break;
         case SCORE_ALGAE_IN_PROCESSOR:
            currentSuperState = CurrentSuperState.SCORE_ALGAE_IN_PROCESSOR;
            break;
         case PREP_CLIMB:
            currentSuperState = CurrentSuperState.PREP_CLIMB;
         case CLIMB:
            currentSuperState = CurrentSuperState.CLIMB;
            break;
         case DEFAULT_STATE:
            if (coralSubsystem.hasCoral()) {
               if (DriverStation.isAutonomous()) {
                  currentSuperState = CurrentSuperState.HOLDING_CORAL_AUTO;
               } else {
                  currentSuperState = CurrentSuperState.HOLDING_CORAL_TELEOP;
               } 
            } else {
               if (DriverStation.isAutonomous()) {
                  currentSuperState = CurrentSuperState.NO_PIECE_AUTO;
               } else {
                  currentSuperState = CurrentSuperState.NO_PIECE_TELEOP;
               }
            }
            break;
      }
      return currentSuperState;
   }

   private void applyStates() {
      switch (currentSuperState) {
         case HOME:
            home();
            break;
         case STOPPED:
            stopped();
            break;
         case NO_PIECE_AUTO:
            noPieceAuto();
            break;
         case NO_PIECE_TELEOP:
            noPiece();
            break;
         case HOLDING_CORAL_TELEOP:
            holdingCoral();
            break;
         case HOLDING_CORAL_AUTO:
            holdingCoralAuto();
            break;
         case HOLDING_ALGAE:
            holdingAlgae();
            break;
         case FORCE_RELOCALIZE:
            relocalizeWithObservationToBeUsed();
            break;
         case SCORE_TELEOP_L1_MANUAL_ALIGNMENT:
            ejectL1(); // could change if we add something. for now, redundant for manual_L1
            break;
         case SCORE_LEFT_TELEOP_L2:
            scoreL2Teleop(Constants.SuperstructureConstants.ScoringSide.LEFT);
            break;
         case SCORE_LEFT_TELEOP_L3:
            scoreL3Teleop(Constants.SuperstructureConstants.ScoringSide.LEFT);
            break;
         case SCORE_LEFT_TELEOP_L4:
            scoreL4Teleop(Constants.SuperstructureConstants.ScoringSide.LEFT);
            break;
         case SCORE_RIGHT_TELEOP_L2:
            scoreL2Teleop(Constants.SuperstructureConstants.ScoringSide.RIGHT);
            break;
         case SCORE_RIGHT_TELEOP_L3:
            scoreL3Teleop(Constants.SuperstructureConstants.ScoringSide.RIGHT);
            break;
         case SCORE_RIGHT_TELEOP_L4:
            scoreL4Teleop(Constants.SuperstructureConstants.ScoringSide.RIGHT);
            break;
         case SCORE_LEFT_AUTO_L2:
            scoreL2Auto(Constants.SuperstructureConstants.ScoringSide.LEFT);
            break;
         case SCORE_LEFT_AUTO_L3:
            scoreL3Auto(Constants.SuperstructureConstants.ScoringSide.LEFT);
            break;
         case SCORE_LEFT_AUTO_L4:
            scoreL4Auto(Constants.SuperstructureConstants.ScoringSide.LEFT);
            break;
         case SCORE_RIGHT_AUTO_L2:
            scoreL2Auto(Constants.SuperstructureConstants.ScoringSide.RIGHT);
            break;
         case SCORE_RIGHT_AUTO_L3:
            scoreL3Auto(Constants.SuperstructureConstants.ScoringSide.RIGHT);
            break;
         case SCORE_RIGHT_AUTO_L4:
            scoreL4Auto(Constants.SuperstructureConstants.ScoringSide.RIGHT);
            break;
         case MANUAL_L4:
            ejectL4();
            break;
         case MANUAL_L3:
            ejectL3();
            break;           
         case MANUAL_L2:
            ejectL2();
            break;  
         case MANUAL_L1:
            ejectL1();
            break;    
         case INTAKE_ALGAE_FROM_REEF_TOP:
            intakeAlgaeFromReefTop();
            break;   
         case INTAKE_ALGAE_FROM_REEF_BOT:
            intakeAlgaeFromReefBot();
            break;  
         case INTAKE_ALGAE_FROM_REEF_GROUND:
            intakeAlgaeFromReefGround();
            break;    
         case INTAKE_CORAL_FROM_STATION:
            intakeCoralFromStation();
            break; 
         case MOVE_ALGAE_TO_PROCESSOR_POSITION:
            moveAlgaeToProcessorPosition();
            break;
         case SCORE_ALGAE_IN_PROCESSOR:
            scoreAlgaeProcessor();
            break;
         case PREP_CLIMB:
            prepClimb();
         case CLIMB:
            climb();
            break;   
      }
   }



    // state machine methods. the following methods are designed to handle all state machine logic.

    private void home() {
        ledSubsystem.setWantedAction(LEDSubsystem.WantedState.DISPLAY_ROBOT_ZERO_ACTION);

        if (elevatorSubsystem.hasHomeCompleted() && previousSuperState == CurrentSuperState.HOME) {
            elevatorSubsystem.setWantedState(ElevatorSubsystem.WantedState.IDLE);
        } else {
            elevatorSubsystem.setWantedState(ElevatorSubsystem.WantedState.HOME);
        }

        if (elevatorSubsystem.hasHomeCompleted() && previousSuperState == CurrentSuperState.HOME) {
            setWantedSuperState(WantedSuperState.DEFAULT_STATE);
        }

        // homing will also, by default, disable the algae subsystem to prevent unnecessary running.
        algaeSubsystem.setWantedState(AlgaeSubsystem.WantedState.IDLE);
     }

     private void stopped() {
        elevatorSubsystem.setWantedState(ElevatorSubsystem.WantedState.IDLE);
        coralSubsystem.setWantedState(CoralSubsystem.WantedState.IDLE);
        algaeSubsystem.setWantedState(AlgaeSubsystem.WantedState.IDLE);
        ledSubsystem.setWantedAction(elevatorSubsystem.hasHomeCompleted() ? LEDSubsystem.WantedState.DISPLAY_ROBOT_ELEVATOR_ZEROED : LEDSubsystem.WantedState.DISPLAY_ROBOT_ELEVATOR_NOT_ZEROED);
     }

     private void holdingAlgae() {
        hasDriveToPointSetPointBeenSet = false;

        swerveSubsystem.setWantedState(SwerveSubsystem.WantedState.TELEOP_DRIVE);
        swerveSubsystem.setTeleopVelocityCoefficient(1.0);
        swerveSubsystem.setRotationVelocityCoefficient(1.0);

        ledSubsystem.setWantedAction(LEDSubsystem.WantedState.DISPLAY_HOLDING_ALGAE);
        
        algaeSubsystem.setWantedState(AlgaeSubsystem.WantedState.HOLD);
        elevatorSubsystem.setWantedState(ElevatorSubsystem.WantedState.HOLD_POSITION);
     }

     private void holdingCoral() {
        coralSpitFlag = false;
        hasDriveToPointSetPointBeenSet = false;

        elevatorSubsystem.setDesiredElevatorSetpoint(Constants.ElevatorConstants.ELEVATOR_L2);

        coralSubsystem.setWantedState(CoralSubsystem.WantedState.PASSIVE_INTAKE);
        algaeSubsystem.setWantedState(AlgaeSubsystem.WantedState.IDLE);

        swerveSubsystem.setWantedState(SwerveSubsystem.WantedState.TELEOP_DRIVE);
        swerveSubsystem.setTeleopVelocityCoefficient(1.0);
        swerveSubsystem.setRotationVelocityCoefficient(1.0);
     }

     private void holdingCoralAuto() {
      elevatorSubsystem.setDesiredElevatorSetpoint(Constants.ElevatorConstants.ELEVATOR_L2);
      algaeSubsystem.setWantedState(AlgaeSubsystem.WantedState.IDLE);
      coralSubsystem.setWantedState(CoralSubsystem.WantedState.PASSIVE_INTAKE);
     }


     private void noPiece() {
        hasDriveToPointSetPointBeenSet = false;
        elevatorSubsystem.setWantedState(ElevatorSubsystem.WantedState.HOLD_POSITION);
        coralSubsystem.setWantedState(CoralSubsystem.WantedState.PASSIVE_INTAKE);
        swerveSubsystem.setWantedState(SwerveSubsystem.WantedState.TELEOP_DRIVE);
        swerveSubsystem.setTeleopVelocityCoefficient(1.0);
        swerveSubsystem.setRotationVelocityCoefficient(1.0);
     }

     private void noPieceAuto() {
        elevatorSubsystem.setDesiredElevatorSetpoint(Constants.ElevatorConstants.ELEVATOR_L1);
        
        coralSubsystem.setWantedState(CoralSubsystem.WantedState.PASSIVE_INTAKE);
     }

     private void relocalizeWithObservationToBeUsed() {
      Logger.recordOutput("Superstructure/IsRelocalizingInAuto", false);
      var observation = getValidTagObservationForCoralScoring();
      if (observation.isPresent()) {
         Logger.recordOutput("Superstructure/IsRelocalizingInAuto", true);
         swerveSubsystem.resetTranslation(observation.get().robotPoseFromCamera());
      }
     }

     // this method handles "move to secondary elevator position" logic during auto.

     private void intakeCoralFromStation() {
      coralSpitFlag = false;
      if (DriverStation.isAutonomous()) {
         elevatorSubsystem.setDesiredElevatorSetpoint(Constants.ElevatorConstants.ELEVATOR_L1);
         coralSubsystem.setWantedState(CoralSubsystem.WantedState.PASSIVE_INTAKE);

         ledSubsystem.setWantedAction(LEDSubsystem.WantedState.DISPLAY_HOLDING_CORAL);
         if (coralSubsystem.hasCoral()) {
          //  elevatorSubsystem.setDesiredElevatorSetpoint(Constants.ElevatorConstants.ELEVATOR_L2);
            coralSubsystem.setWantedState(CoralSubsystem.WantedState.PASSIVE_INTAKE);
            if (elevatorSubsystem.reachedSetpoint()) { 
               elevatorSubsystem.setWantedState(ElevatorSubsystem.WantedState.HOLD_POSITION);
            }
         }
      } else {
         if (coralSubsystem.hasCoral()) {
            ledSubsystem.setWantedAction(LEDSubsystem.WantedState.DISPLAY_HOLDING_CORAL);
            swerveSubsystem.setState(SwerveSubsystem.WantedState.TELEOP_DRIVE);
         } else if (RobotState.getInstance().getRobotPoseFromSwerveDriveOdometry().getRotation().getDegrees() >= 0) {
            var angleToSnapTo = Field.isBlueAlliance() ? 126.0 : 54.0; // these values are incorrect for our robot
            swerveSubsystem.setTargetRotation(Rotation2d.fromDegrees(angleToSnapTo));
         } else {
            var angleToSnapTo = Field.isBlueAlliance() ? -126.0 : -54.0; // these values are incorrect for our robot
            swerveSubsystem.setTargetRotation((Rotation2d.fromDegrees(angleToSnapTo)));
         }
      } 
     }

     private void intakeAlgaeFromReefTop() {
      elevatorSubsystem.setDesiredElevatorSetpoint(Constants.ElevatorConstants.ELEVATOR_ALGAE_TOP);
      algaeSubsystem.setWantedState(AlgaeSubsystem.WantedState.INTAKE);
      if (elevatorSubsystem.reachedSetpoint()) {
         elevatorSubsystem.setWantedState(ElevatorSubsystem.WantedState.HOLD_POSITION);
      }
     }

     private void intakeAlgaeFromReefBot() {
      elevatorSubsystem.setDesiredElevatorSetpoint(Constants.ElevatorConstants.ELEVATOR_ALGAE_BOT);
      algaeSubsystem.setWantedState(AlgaeSubsystem.WantedState.INTAKE);
      if (elevatorSubsystem.reachedSetpoint()) {
         elevatorSubsystem.setWantedState(ElevatorSubsystem.WantedState.HOLD_POSITION);
      }
     }

     private void prepClimb() {
       elevatorSubsystem.setDesiredElevatorSetpoint(Constants.ElevatorConstants.ELEVATOR_L2);
       swerveSubsystem.setTeleopVelocityCoefficient(0.5); // reduce speed while held

      //elevatorSubsystem.setWantedState(ElevatorSubsystem.WantedState.OPEN_SERVO);
     }

     private void intakeAlgaeFromReefGround() {
      elevatorSubsystem.setDesiredElevatorSetpoint(Constants.ElevatorConstants.ELEVATOR_ALGAE_GROUND);
      algaeSubsystem.setWantedState(AlgaeSubsystem.WantedState.INTAKE);
      if (elevatorSubsystem.reachedSetpoint()) {
         elevatorSubsystem.setWantedState(ElevatorSubsystem.WantedState.HOLD_POSITION);
      }
     }

     private void finishIntakingAlgaeAndHold() {
      elevatorSubsystem.setDesiredElevatorSetpoint(elevatorSubsystem.getCurrentElevatorPositionRotations() + 0.1); // make a constant
      algaeSubsystem.setWantedState(AlgaeSubsystem.WantedState.HOLD);
     }

     private void ejectL1() {
      coralSubsystem.setWantedState(CoralSubsystem.WantedState.SPIT_L1);
     }

     // redundancy; accuracy may be improved by changing voltages on a per-level basis (at least for L4)

     private void ejectL2() {
      coralSubsystem.setWantedState(CoralSubsystem.WantedState.SPIT);
     }

     private void ejectL3() {
      coralSubsystem.setWantedState(CoralSubsystem.WantedState.SPIT);
     }

     private void ejectL4() {
      coralSubsystem.setWantedState(CoralSubsystem.WantedState.SPIT);
     }

     private void scoreL2Teleop(Constants.SuperstructureConstants.ScoringSide side) {
      scoringLevel = Constants.SuperstructureConstants.ScoringLevel.L2;
      scoringSide = side;

      var isObservationPresent = driveToScoringPoseAndReturnIfObservationPresent();

      if (isObservationPresent) {
         elevatorSubsystem.setDesiredElevatorSetpoint(Constants.ElevatorConstants.ELEVATOR_L2);
         coralSubsystem.setWantedState(CoralSubsystem.WantedState.PASSIVE_INTAKE);

         if (elevatorSubsystem.reachedSetpoint()) {
            elevatorSubsystem.setWantedState(ElevatorSubsystem.WantedState.HOLD_POSITION);
         }

      } if (isReadyToEjectInTeleopPeriod() && isObservationPresent) {
         coralSpitFlag = true;
      }

      if (coralSpitFlag) {
         if (automationLevel == Constants.SuperstructureConstants.AutomationLevel.AUTO_RELEASE) {
            coralSubsystem.setWantedState(CoralSubsystem.WantedState.SPIT);
         }
      }

      if (!hasCoral()) {
         setWantedSuperState(WantedSuperState.DEFAULT_STATE);
      }
   }

   private void scoreL2Auto(Constants.SuperstructureConstants.ScoringSide scoringSide) {
      scoringLevel = Constants.SuperstructureConstants.ScoringLevel.L2;

      elevatorSubsystem.setDesiredElevatorSetpoint(Constants.ElevatorConstants.ELEVATOR_L2);

      if (elevatorSubsystem.reachedSetpoint()) {
         elevatorSubsystem.setWantedState(ElevatorSubsystem.WantedState.HOLD_POSITION);
      } 

      if (isReadyToEjectInAutoPeriod()) {
         coralSpitFlag = true;
      }

      if (coralSpitFlag) {
         coralSubsystem.setWantedState(CoralSubsystem.WantedState.SPIT);
      }

      relocalizeWithObservationToBeUsed();
   }


   private void scoreL3Teleop(Constants.SuperstructureConstants.ScoringSide side) {
      scoringLevel = Constants.SuperstructureConstants.ScoringLevel.L3;
      scoringSide = side;

      var isObservationPresent = driveToScoringPoseAndReturnIfObservationPresent();

      if (isObservationPresent) {
         elevatorSubsystem.setDesiredElevatorSetpoint(Constants.ElevatorConstants.ELEVATOR_L3);
         coralSubsystem.setWantedState(CoralSubsystem.WantedState.PASSIVE_INTAKE);

         if (elevatorSubsystem.reachedSetpoint()) {
            elevatorSubsystem.setWantedState(ElevatorSubsystem.WantedState.HOLD_POSITION);
         }

      } if (isReadyToEjectInTeleopPeriod() && isObservationPresent) {
         coralSpitFlag = true;
      }

      if (coralSpitFlag) {
         if (automationLevel == Constants.SuperstructureConstants.AutomationLevel.AUTO_RELEASE) {
            coralSubsystem.setWantedState(CoralSubsystem.WantedState.SPIT);
         }
      }

      if (!hasCoral()) {
         setWantedSuperState(WantedSuperState.DEFAULT_STATE);
      }
   }

   private void scoreL3Auto(Constants.SuperstructureConstants.ScoringSide scoringSide) {
      scoringLevel = Constants.SuperstructureConstants.ScoringLevel.L3;

      elevatorSubsystem.setDesiredElevatorSetpoint(Constants.ElevatorConstants.ELEVATOR_L3);

      if (elevatorSubsystem.reachedSetpoint()) {
         elevatorSubsystem.setWantedState(ElevatorSubsystem.WantedState.HOLD_POSITION);
      } 

      if (isReadyToEjectInAutoPeriod()) {
         coralSpitFlag = true;
      }

      if (coralSpitFlag) {
         coralSubsystem.setWantedState(CoralSubsystem.WantedState.SPIT);
      }

      relocalizeWithObservationToBeUsed();
   }


   private void scoreL4Teleop(Constants.SuperstructureConstants.ScoringSide side) {
      scoringLevel = Constants.SuperstructureConstants.ScoringLevel.L4;
      scoringSide = side;

      var isObservationPresent = driveToScoringPoseAndReturnIfObservationPresent();

      if (isObservationPresent) {
         elevatorSubsystem.setDesiredElevatorSetpoint(Constants.ElevatorConstants.ELEVATOR_L4);
         coralSubsystem.setWantedState(CoralSubsystem.WantedState.PASSIVE_INTAKE);

         if (elevatorSubsystem.reachedSetpoint()) {
            elevatorSubsystem.setWantedState(ElevatorSubsystem.WantedState.HOLD_POSITION);
         }

      } if (isReadyToEjectInTeleopPeriod() && isObservationPresent) {
         coralSpitFlag = true;
      }

      if (coralSpitFlag) {
         if (automationLevel == Constants.SuperstructureConstants.AutomationLevel.AUTO_RELEASE) {
            coralSubsystem.setWantedState(CoralSubsystem.WantedState.SPIT);
         }
      }

      if (!hasCoral()) {
         setWantedSuperState(WantedSuperState.DEFAULT_STATE);
      }
   }

   private void scoreL4Auto(Constants.SuperstructureConstants.ScoringSide scoringSide) {
      scoringLevel = Constants.SuperstructureConstants.ScoringLevel.L4;

      elevatorSubsystem.setDesiredElevatorSetpoint(Constants.ElevatorConstants.ELEVATOR_L4);

      if (elevatorSubsystem.reachedSetpoint()) {
         elevatorSubsystem.setWantedState(ElevatorSubsystem.WantedState.HOLD_POSITION);
      } 

      if (isReadyToEjectInAutoPeriod()) {
         coralSpitFlag = true;
      }

      if (coralSpitFlag) {
         coralSubsystem.setWantedState(CoralSubsystem.WantedState.SPIT);
      }

      relocalizeWithObservationToBeUsed();
   }

   private void moveAlgaeToProcessorPosition() {
      var rotation = Field.isBlueAlliance() ? Rotation2d.kCW_90deg : Rotation2d.kCCW_90deg; // probably needs to be swapped, will test
      algaeSubsystem.setWantedState(AlgaeSubsystem.WantedState.HOLD);
      if (RobotState.getInstance().getRobotPoseFromSwerveDriveOdometry().getRotation()
      .getDegrees() > 0) {
         rotation = Rotation2d.kCCW_90deg; // might need to be swapped
      } else {
         rotation = Rotation2d. kCW_90deg;
      }

      swerveSubsystem.setTargetRotation(rotation);
      elevatorSubsystem.setDesiredElevatorSetpoint(Constants.ElevatorConstants.ELEVATOR_ALGAE_TEE);

      if (elevatorSubsystem.reachedSetpoint()) {
         elevatorSubsystem.setWantedState(ElevatorSubsystem.WantedState.HOLD_POSITION);
      }
      ledSubsystem.setWantedAction(LEDSubsystem.WantedState.DISPLAY_ALIGN_TO_PROCESSOR);
   }

   private void scoreAlgaeProcessor() {
      algaeSubsystem.setWantedState(AlgaeSubsystem.WantedState.SPIT);

      // led check  
      if (!algaeSubsystem.isHolding()) {
         ledSubsystem.setWantedAction(LEDSubsystem.WantedState.DISPLAY_TAG_NOT_SEEN); 
      }
   }

   private void climb() {

         elevatorSubsystem.setWantedState(ElevatorSubsystem.WantedState.CLIMB);
         // disable all unnecessary subsystems
         coralSubsystem.setWantedState(CoralSubsystem.WantedState.IDLE);
         algaeSubsystem.setWantedState(AlgaeSubsystem.WantedState.IDLE);
         // flash
         ledSubsystem.setWantedAction(LEDSubsystem.WantedState.DISPLAY_CLIMBING);
   }



     // Superstructure methods.

     public void setWantedSuperState(WantedSuperState superState) {
        this.wantedSuperState = superState;
     }


     public Command setStateCommand(WantedSuperState superState) {
      Command commandToReturn = new InstantCommand(() -> setWantedSuperState(superState));
      // may need logic here that checks the servo position before running the command
      return commandToReturn;
   }

     public Command setStateCommandWithServoCheck(WantedSuperState superState, boolean servo) {
        Command commandToReturn = new InstantCommand(() -> setWantedSuperState(superState));
        // may need logic here that checks the servo position before running the command
        return commandToReturn;
     }

     public boolean hasCoral() {
        return coralSubsystem.hasCoral();
     }



     public Optional<RobotState.AprilTagObservation> getValidTagObservationForCoralScoring() {
      Optional<RobotState.AprilTagObservation> observationToBeUsed = Optional.empty();

      if (reefSelectionMethod == Constants.SuperstructureConstants.ReefSelectionMethod.ROTATION || DriverStation.isAutonomous()) {
         var validTagIds = RobotState.getInstance().getValidTagIDsFromClosest60DegreeRotation();

         for (var observation : RobotState.getInstance().getAprilTagObservations()) {
            // because we only utilize two same-side cameras, we don't really care about front/back
            // this is a bit riskier than 2910 implementation, but we may see reduced error rates for alignment when we are in the "middle" of two tags
            if (observation.tagId() == validTagIds.FRONT_ID || observation.tagId() == validTagIds.BACK_ID) {
               observationToBeUsed = Optional.of(observation);
            }
         }
         
         Logger.recordOutput(
            "Superstructure/AprilTagObservationToBeUsed/isPresent", observationToBeUsed.isPresent());
         if (observationToBeUsed.isPresent()) {
         Logger.recordOutput(
                "Superstructure/AprilTagObservationToBeUsed/robotPoseFromCamera",
                observationToBeUsed.get().robotPoseFromCamera());
         Logger.recordOutput(
                "Superstructure/AprilTagObservationToBeUsed/tagId",
                observationToBeUsed.get().tagId());
         Logger.recordOutput(
                "Superstructure/AprilTagObservationToBeUsed/cameraName",
                observationToBeUsed.get().cameraName());
    
         }
     } else {
      var desiredId = RobotState.getInstance().getClosestTagId();
      for (var observation : RobotState.getInstance().getAprilTagObservations()) {
         if (observation.tagId() == desiredId) {
            observationToBeUsed = Optional.of(observation);
         }
      }

      Logger.recordOutput(
         "Superstructure/AprilTagObservationToBeUsed/isPresent", observationToBeUsed.isPresent());
      if (observationToBeUsed.isPresent()) {
      Logger.recordOutput(
             "Superstructure/AprilTagObservationToBeUsed/robotPoseFromCamera",
             observationToBeUsed.get().robotPoseFromCamera());
      Logger.recordOutput(
             "Superstructure/AprilTagObservationToBeUsed/tagId",
             observationToBeUsed.get().tagId());
      Logger.recordOutput(
             "Superstructure/AprilTagObservationToBeUsed/cameraName",
             observationToBeUsed.get().cameraName());
 
      }
     }
     return observationToBeUsed;
   }

   public boolean isReadyToEjectInTeleopPeriod() {
      return swerveSubsystem.isAtDriveToPointSetpoint()
              && swerveSubsystem.isAtDesiredRotation(Units.degreesToRadians(2.0))
              && elevatorSubsystem.reachedSetpoint();
  }

  public boolean isReadyToEjectInAutoPeriod() {
   return elevatorSubsystem.reachedSetpoint() && swerveSubsystem.isAtEndOfChoreoTrajectoryOrDriveToPoint();
}



   public boolean driveToScoringPoseAndReturnIfObservationPresent() {
      var observationToBeUsed = getValidTagObservationForCoralScoring();
  
      if (observationToBeUsed.isEmpty()) {
          ledSubsystem.setWantedAction(LEDSubsystem.WantedState.DISPLAY_TAG_NOT_SEEN);
          swerveSubsystem.setTargetRotation(RobotState.getInstance().getClosest60Degrees());
          hasDriveToPointSetPointBeenSet = false;
          return false;
      }
  
      var observation = observationToBeUsed.get();
      swerveSubsystem.resetTranslation(observation.robotPoseFromCamera());
  
      Pose2d desiredPoseToDriveTo = Field.getDesiredPointToDriveToForCoralScoring(
          observation.tagId(), scoringSide, scoringLevel);
  
      Logger.recordOutput("Superstructure/DesiredPointToDriveTo", desiredPoseToDriveTo);
  
      if (reefSelectionMethod == Constants.SuperstructureConstants.ReefSelectionMethod.POSE) {
          ledSubsystem.setWantedAction(LEDSubsystem.WantedState.DISPLAY_OFF);
          swerveSubsystem.setDesiredPoseForDriveToPoint(desiredPoseToDriveTo);
      } else {
          swerveSubsystem.setDesiredPoseForDriveToPointWithMaximumAngularVelocity(desiredPoseToDriveTo, 3.0);
      }
  
      hasDriveToPointSetPointBeenSet = true;
      return true;
     }

     public Command configureButtonBinding(
      WantedSuperState hasCoralCondition,
      WantedSuperState hasAlgaeCondition,
      WantedSuperState noPieceCondition) {
            return Commands.either(
            setStateCommand(hasAlgaeCondition),
        Commands.either(
            setStateCommand(hasCoralCondition),
            setStateCommand(noPieceCondition),
            coralSubsystem::hasCoral),
        () -> algaeModeActive);
         }


      public void toggleReefSelectionMethod() {
      reefSelectionMethod = (reefSelectionMethod == Constants.SuperstructureConstants.ReefSelectionMethod.POSE)
              ? Constants.SuperstructureConstants.ReefSelectionMethod.ROTATION
              : Constants.SuperstructureConstants.ReefSelectionMethod.POSE;
  }

  public void toggleAlgaeMode() {
   algaeModeActive = (algaeModeActive == false ? true : false);
  }


  public void setDesiredReefLevelForDriveToPoint(Constants.SuperstructureConstants.ScoringLevel level) {
   scoringLevel = level;
  }

  public void toggleHasPoseBeenSetForPrematch(boolean hasPoseBeenResetPrematch) {
   this.hasPoseBeenResetPrematch = hasPoseBeenResetPrematch;
}

  public void setDesiredScoringSideForDriveToPoint(Constants.SuperstructureConstants.ScoringSide side) {
   scoringSide = side;
  }
}
