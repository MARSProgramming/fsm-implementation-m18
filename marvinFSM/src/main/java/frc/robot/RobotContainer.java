// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
