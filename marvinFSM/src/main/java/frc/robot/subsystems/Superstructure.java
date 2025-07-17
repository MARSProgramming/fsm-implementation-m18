package frc.robot.subsystems;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotState;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.SuperstructureConstants;
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
        FORCE_RELOCALIZE_LEFT,
        FORCE_RELOCALIZE_RIGHT,
        SCORE_L1_MANUAL_ALIGN,
        SCORE_L1_LEFT_BASE,
        SCORE_L1_RIGHT_BASE,
        SCORE_L1_LEFT_TOP,
        SCORE_L1_RIGHT_TOP,
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
        INTAKE_ALGAE_FROM_REEF_TOP,
        INTAKE_ALGAE_FROM_REEF_BOT,
        MOVE_ALGAE_TO_PROCESSOR_POSITION,
        SCORE_ALGAE_IN_PROCESSOR,
        CLIMB
    }

    public enum CurrentSuperState {
        HOME,
        STOPPED,
        HOLDING_CORAL_TELEOP,
        NO_PIECE_AUTO,
        HOLDING_CORAL_AUTO,
        HOLDING_ALGAE,
        FORCE_RELOCALIZE_LEFT,
        FORCE_RELOCALIZE_RIGHT,
        SCORE_TELEOP_L1_MANUAL_ALIGNMENT,
        SCORE_LEFT_TELEOP_L2,
        SCORE_LEFT_TELEOP_L3,
        SCORE_LEFT_TELEOP_L4,
        SCORE_RIGHT_TELEOP_L2,
        SCORE_RIGHT_TELEOP_L3,
        SCORE_RIGHT_TELEOP_L4,
        SCORE_AUTO_L1,
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
        INTAKE_ALGAE_FROM_HP,
        INTAKE_ALGAE_FROM_REEF_TOP,
        INTAKE_ALGAE_FROM_REEF_BOT,
        MOVE_ALGAE_TO_PROCESSOR_POSITION,
        SCORE_ALGAE_IN_PROCESSOR,
        CLIMB
    }

    private WantedSuperState wantedSuperState = WantedSuperState.STOPPED;
    private CurrentSuperState currentSuperState = CurrentSuperState.STOPPED;
    private CurrentSuperState previousSuperState;

    private boolean hasDriveToPointSetPointBeenSet = false;

    private NeutralModeValue pastSwitchValue = NeutralModeValue.Brake;

    private boolean hasPoseBeenResetPrematch = false;

    private boolean allowExternalCommandToAccessLEDS = false;

    private boolean hasElevatorReachedSetpoint = false;
    private boolean hasCoral = false;
    private boolean algaeMotorStalling = false;    

    public Superstructure(SwerveSubsystem swerve, CoralSubsystem coral, AlgaeSubsystem algae, ElevatorSubsystem elev, LEDSubsystem led, Dash dash) {
        swerveSubsystem = swerve;
        coralSubsystem = coral;
        algaeSubsystem = algae;
        elevatorSubsystem = elev;
        ledSubsystem = led;
        operatorDashboard = dash;
    }



    // state machine methods. the following methods are designed to handle all state machine logic.

    private void home() {
        ledSubsystem.setWantedAction(LEDSubsystem.WantedState.DISPLAY_ROBOT_ZERO_ACTION);

        if (elevatorSubsystem.hasHomeCompleted() && previousSuperState == CurrentSuperState.HOME) {
            elevatorSubsystem.setWantedState(ElevatorSubsystem.WantedState.IDLE);
        } else {
            elevatorSubsystem.setWantedState(ElevatorSubsystem.WantedState.HOME);
        }

        if (elevatorSubsystem.hasHomeCompleted() && previousSuperState == currentSuperState.HOME) {
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
        elevatorSubsystem.setDesiredElevatorSetpoint(Constants.ElevatorConstants.ELEVATOR_ALGAE_TEE);

        swerveSubsystem.setWantedState(SwerveSubsystem.WantedState.TELEOP_DRIVE);
        swerveSubsystem.setTeleopVelocityCoefficient(1.0);
        swerveSubsystem.setRotationVelocityCoefficient(1.0);

        ledSubsystem.setWantedAction(LEDSubsystem.WantedState.DISPLAY_HOLDING_ALGAE);
        
        algaeSubsystem.setWantedState(AlgaeSubsystem.WantedState.HOLD);
     }

     private void holdingCoral() {
         coralSpitFlag = false;
        hasDriveToPointSetPointBeenSet = false;

        elevatorSubsystem.setDesiredElevatorSetpoint(Constants.ElevatorConstants.ELEVATOR_L1);

        coralSubsystem.setWantedState(CoralSubsystem.WantedState.PASSIVE_INTAKE);

        swerveSubsystem.setWantedState(SwerveSubsystem.WantedState.TELEOP_DRIVE);
        swerveSubsystem.setTeleopVelocityCoefficient(1.0);
        swerveSubsystem.setRotationVelocityCoefficient(1.0);
     }

     private void holdingCoralAuto() {
        elevatorSubsystem.setDesiredElevatorSetpoint(Constants.ElevatorConstants.ELEVATOR_L1);
        coralSubsystem.setWantedState(CoralSubsystem.WantedState.PASSIVE_INTAKE);
     }


     private void noPiece() {
        hasDriveToPointSetPointBeenSet = false;
        elevatorSubsystem.setDesiredElevatorSetpoint(Constants.ElevatorConstants.ELEVATOR_L1);
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
            elevatorSubsystem.setDesiredElevatorSetpoint(Constants.ElevatorConstants.ELEVATOR_L2);
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
      if (algaeSubsystem.isHolding()) {
         algaeSubsystem.setWantedState(AlgaeSubsystem.WantedState.SPIT);
      } else {
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
                Commands.either(
                  setStateCommand(hasCoralCondition),
                  setStateCommand(hasAlgaeCondition),
                  coralSubsystem::hasCoral),
                setStateCommand(noPieceCondition),
                () -> coralSubsystem.hasCoral() || algaeSubsystem.isHolding());
          }





      public void toggleReefSelectionMethod() {
      reefSelectionMethod = (reefSelectionMethod == Constants.SuperstructureConstants.ReefSelectionMethod.POSE)
              ? Constants.SuperstructureConstants.ReefSelectionMethod.ROTATION
              : Constants.SuperstructureConstants.ReefSelectionMethod.POSE;
  }

  public void setDesiredReefLevelForDriveToPoint(Constants.SuperstructureConstants.ScoringLevel level) {
   scoringLevel = level;
  }

  public void setDesiredScoringSideForDriveToPoint(Constants.SuperstructureConstants.ScoringSide side) {
   scoringSide = side;
  }
}
