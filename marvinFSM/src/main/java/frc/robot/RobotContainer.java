// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.algae.AlgaeSubsystem;
import frc.robot.subsystems.algae.GrabberIOSparkMax;
import frc.robot.subsystems.coral.CoralSubsystem;
import frc.robot.subsystems.coral.SensorIORoboRIO;
import frc.robot.subsystems.coral.SpitterIOTalonSRX;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.LimiterIORoboRIO;
import frc.robot.subsystems.led.LEDIOCandle;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.swerve.SwerveIOCTRE;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.util.Dash;

public class RobotContainer {
  private final SwerveSubsystem swerveSubsystem;
  private final VisionSubsystem visionSubsystem;
  private final ElevatorSubsystem elevatorSubsystem;
  private final CoralSubsystem coralSubsystem;
  private final AlgaeSubsystem algaeSubsystem;
  private final LEDSubsystem ledSubsystem;
  private final Dash operatorDashboard;

  private final Superstructure superstructure;

  private final CommandXboxController controller = new CommandXboxController(0);

  public RobotContainer() {
    SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>[] moduleConstants = DriveConstants.getModuleConstants();

    swerveSubsystem = new SwerveSubsystem(
      new SwerveIOCTRE(DriveConstants.getSwerveDrivetrainConstants(), DriveConstants.getModuleConstants()),
      controller,
      moduleConstants[0].SpeedAt12Volts,
      moduleConstants[0].SpeedAt12Volts
              / Math.hypot(moduleConstants[0].LocationX, moduleConstants[0].LocationY)); 
              
      coralSubsystem = new CoralSubsystem(new SpitterIOTalonSRX(), new SensorIORoboRIO());
      algaeSubsystem = new AlgaeSubsystem(new GrabberIOSparkMax());
      elevatorSubsystem = new ElevatorSubsystem(new ElevatorIOTalonFX(), new LimiterIORoboRIO());
      visionSubsystem = new VisionSubsystem(new VisionIOPhotonVision("reef_cam", VisionConstants.robotToCamera0), new VisionIOPhotonVision("feeder_cam", VisionConstants.robotToCamera1));
      ledSubsystem = new LEDSubsystem(new LEDIOCandle(0, "rio"));

      operatorDashboard = new Dash();

      superstructure = new Superstructure(swerveSubsystem, coralSubsystem, algaeSubsystem, elevatorSubsystem, ledSubsystem, operatorDashboard);

      configureBindings();
      }

  private void configureBindings() {
    controller.leftStick().onTrue(instantCommand(superstructure::toggleReefSelectionMethod));

    controller
              .leftTrigger()
              .onTrue(superstructure.configureButtonBinding(
                Superstructure.WantedSuperState.SCORE_LEFT_L2, // Move the elevator to LEFT L2 position and score depending on automation.
                Superstructure.WantedSuperState.MOVE_ALGAE_TO_PROCESSOR_POSITION, // Moves the elevator to processor position if we have algae.
                Superstructure.WantedSuperState.INTAKE_ALGAE_FROM_REEF_GROUND // Moves the elevator to intaking algae from the ground.
                ))
                .onFalse(superstructure.setStateCommand(Superstructure.WantedSuperState.DEFAULT_STATE));

    controller
              .rightTrigger()
              .onTrue(superstructure.configureButtonBinding(
                Superstructure.WantedSuperState.SCORE_RIGHT_L2, // Move the elevator to RIGHT L2 position and score depending on automation.
                Superstructure.WantedSuperState.SCORE_ALGAE_IN_PROCESSOR, // Process algae.
                Superstructure.WantedSuperState.HOME // Home if we dont have a piece or are holding algae.
                ))
                .onFalse(superstructure.setStateCommand(Superstructure.WantedSuperState.DEFAULT_STATE));

    controller
              .leftBumper()
              .onTrue(superstructure.configureButtonBinding(
                Superstructure.WantedSuperState.SCORE_LEFT_L4, // Move the elevator to a LEFT L4 position and score depending on automation.
                Superstructure.WantedSuperState.HOLD_ALGAE, // Holds algae.
                Superstructure.WantedSuperState.INTAKE_ALGAE_FROM_REEF_TOP // Moves the elevator to intaking algae from the top of reef. 
                ))
                .onFalse(superstructure.setStateCommand(Superstructure.WantedSuperState.DEFAULT_STATE));

    controller
              .rightBumper()
              .onTrue(superstructure.configureButtonBinding(
                Superstructure.WantedSuperState.SCORE_RIGHT_L4, // Move the elevator to a RIGHT L4 position and score depending on automation.
                Superstructure.WantedSuperState.HOLD_ALGAE, // Holds algae.
                Superstructure.WantedSuperState.INTAKE_ALGAE_FROM_REEF_BOT // Align the drivetrain for intaking coral.
                ))
                .onFalse(superstructure.setStateCommand(Superstructure.WantedSuperState.DEFAULT_STATE));


    controller
              .leftTrigger()
              .and(controller.povLeft())
              .onTrue(superstructure.configureButtonBinding(
                Superstructure.WantedSuperState.SCORE_LEFT_L3, // Move the elevator to a LEFT L3 position and score depending on automation.
                Superstructure.WantedSuperState.HOLD_ALGAE, // Continues to hold algae.
                Superstructure.WantedSuperState.HOME // Homes elevator if no pieces are detected.
                ))
                .onFalse(superstructure.setStateCommand(Superstructure.WantedSuperState.DEFAULT_STATE));

    controller
              .rightTrigger()
              .and(controller.povRight())
              .onTrue(superstructure.configureButtonBinding(
                Superstructure.WantedSuperState.SCORE_RIGHT_L3, // Move the elevator to a RIGHT L3 position and score depending on automation.
                Superstructure.WantedSuperState.HOLD_ALGAE, // Continues to hold algae.
                Superstructure.WantedSuperState.HOME // Homes elevator if no pieces are detected.
                ))
                .onFalse(superstructure.setStateCommand(Superstructure.WantedSuperState.DEFAULT_STATE));

    controller
              .x()
              .onTrue(superstructure.configureButtonBinding(
                Superstructure.WantedSuperState.MANUAL_L4, // Spits coral at any level manually. (allows for L1 scoring)
                Superstructure.WantedSuperState.HOLD_ALGAE, // Continues to hold algae.
                Superstructure.WantedSuperState.MANUAL_L4 // Spit coral at any level if sensor is not working.
                ))
                .onFalse(superstructure.setStateCommand(Superstructure.WantedSuperState.DEFAULT_STATE));

    
    controller
              .b()
              .onTrue(superstructure.configureButtonBinding(
                Superstructure.WantedSuperState.SCORE_ALGAE_IN_PROCESSOR, // Force eject algae.
                Superstructure.WantedSuperState.SCORE_ALGAE_IN_PROCESSOR, // Force eject algae.
                Superstructure.WantedSuperState.SCORE_ALGAE_IN_PROCESSOR // Force eject algae.
                ))
                .onFalse(superstructure.setStateCommand(Superstructure.WantedSuperState.DEFAULT_STATE));

    controller
              .a()
              .onTrue(superstructure.configureButtonBinding(
                Superstructure.WantedSuperState.HOLD_ALGAE, // Force hold algae.
                Superstructure.WantedSuperState.HOLD_ALGAE, // Force hold algae.
                Superstructure.WantedSuperState.HOLD_ALGAE // Force hold algae.
                ))
                .onFalse(superstructure.setStateCommand(Superstructure.WantedSuperState.DEFAULT_STATE));

    controller
              .a()
              .and(controller.povUp())
              .onTrue(superstructure.configureButtonBinding(
                Superstructure.WantedSuperState.FORCE_RELOCALIZE, // Force a relocalization of the drivetrain.
                Superstructure.WantedSuperState.FORCE_RELOCALIZE, // Force a relocalization of the drivetrain.
                Superstructure.WantedSuperState.FORCE_RELOCALIZE // Force a relocalization of the drivetrain.
                ))
                .onFalse(superstructure.setStateCommand(Superstructure.WantedSuperState.DEFAULT_STATE));

    controller
              .y()
              .onTrue(superstructure.configureButtonBinding(
                Superstructure.WantedSuperState.PREP_CLIMB, // Prepare for climbing sequence.
                Superstructure.WantedSuperState.PREP_CLIMB, // Prepare for climbing sequence.
                Superstructure.WantedSuperState.PREP_CLIMB // Prepare for climbing sequence.
                ))
                .onFalse(superstructure.setStateCommand(Superstructure.WantedSuperState.DEFAULT_STATE));

    controller
              .povUp()
              .and(controller.povDown()) // This configuration makes it difficult to accidentally initiate a climb.
              .onTrue(superstructure.configureButtonBinding(
                Superstructure.WantedSuperState.CLIMB, // Initiate climbing sequence.
                Superstructure.WantedSuperState.CLIMB, // Initiate climbing sequence.
                Superstructure.WantedSuperState.CLIMB // Initiate climbing sequence.
                ))
                .onFalse(superstructure.setStateCommand(Superstructure.WantedSuperState.DEFAULT_STATE));
    



  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  private static InstantCommand instantCommand(Runnable runnable) {
    return new InstantCommand(runnable) {
        @Override
        public boolean runsWhenDisabled() {
            return true;
        }
    };
}

  
}
