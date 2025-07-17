package frc.robot.constants;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.util.Units;
import frc.robot.util.CanDeviceId;

import frc.robot.util.ConfigureSlot0Gains;


public class DriveConstants {
    private static final String CANIVORE_CANBUS_NAME = "CAN-2";
    // Ports and IDs
    private static final CanDeviceId GYRO = new CanDeviceId(20, CANIVORE_CANBUS_NAME);

    // Swerve Modules
    private static final CanDeviceId FRONT_LEFT_DRIVE_MOTOR = new CanDeviceId(2, CANIVORE_CANBUS_NAME);
    private static final CanDeviceId FRONT_LEFT_STEER_MOTOR = new CanDeviceId(3, CANIVORE_CANBUS_NAME);
    private static final CanDeviceId FRONT_LEFT_STEER_ENCODER = new CanDeviceId(25, CANIVORE_CANBUS_NAME);

    private static final CanDeviceId FRONT_RIGHT_DRIVE_MOTOR = new CanDeviceId(8, CANIVORE_CANBUS_NAME);
    private static final CanDeviceId FRONT_RIGHT_STEER_MOTOR = new CanDeviceId(9, CANIVORE_CANBUS_NAME);
    private static final CanDeviceId FRONT_RIGHT_STEER_ENCODER = new CanDeviceId(26, CANIVORE_CANBUS_NAME);

    private static final CanDeviceId BACK_LEFT_DRIVE_MOTOR = new CanDeviceId(4, CANIVORE_CANBUS_NAME);
    private static final CanDeviceId BACK_LEFT_STEER_MOTOR = new CanDeviceId(5, CANIVORE_CANBUS_NAME);
    private static final CanDeviceId BACK_LEFT_STEER_ENCODER = new CanDeviceId(28, CANIVORE_CANBUS_NAME);

    private static final CanDeviceId BACK_RIGHT_DRIVE_MOTOR = new CanDeviceId(6, CANIVORE_CANBUS_NAME);
    private static final CanDeviceId BACK_RIGHT_STEER_MOTOR = new CanDeviceId(7, CANIVORE_CANBUS_NAME);
    private static final CanDeviceId BACK_RIGHT_STEER_ENCODER = new CanDeviceId(27, CANIVORE_CANBUS_NAME);

    /**
     * Wheel radius in meters. Accuracy in these measurements affects wheel odometry
     * which measures distance as a function of the number of rotations * wheel circumference.
     */
    private static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(2); // Old value 1.95

    /** Ratio between the drive motor shaft and the output shaft the wheel is mounted on. */
    private static final double DRIVE_GEAR_RATIO = 5.357142857142857;

    /** Ratio between the steer motor shaft and the steer output shaft. */

    private static final double MK4n_STEER_GEAR_RATIO = 18.75;

    /**
     * The coupled gear ratio between the CanCoder and the drive motor.
     * Every 1 rotation of the steer motor results in coupled ratio of drive turns.
     */
    private static final double COUPLING_GEAR_RATIO = 3.125;

    /**
     * Wheelbase length is the distance between the front and back wheels.
     * Positive x values represent moving towards the front of the robot
     */
    private static final double WHEELBASE_LENGTH_METERS = Units.inchesToMeters(22.75);

    /**
     * Wheel track width is the distance between the left and right wheels.
     * Positive y values represent moving towards the left of the robot.
     */
    private static final double WHEEL_TRACK_WIDTH_METERS = Units.inchesToMeters(22.75);

    /**
     * The maximum speed of the robot in meters per second.
     */
    private static final double MAX_SPEED_METERS_PER_SECOND = Units.feetToMeters(16.0);

    // CANcoder offsets of the swerve modules
    private static final double FRONT_LEFT_STEER_OFFSET_ROTATIONS = 0.2041015625;
    // -Rotation2d.fromRadians(1.342).getRotations();
    private static final double FRONT_RIGHT_STEER_OFFSET_ROTATIONS = 0.446044921875;
    // -Rotation2d.fromRadians(-1.519).getRotations();
    private static final double BACK_LEFT_STEER_OFFSET_ROTATIONS = 0.400634765625;
    // -Rotation2d.fromRadians(-2.586).getRotations();
    private static final double BACK_RIGHT_STEER_OFFSET_ROTATIONS = -0.211669921875;
    // -Rotation2d.fromRadians(0.550).getRotations();

    // private static final double GYRO_ERROR = 0.28; how can we  compute this?

    // Robot configuration
    private static SwerveDrivetrainConstants swerveDrivetrainConstants;
    private static SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>[]
            moduleConstants;

    @SuppressWarnings("unchecked")
    public DriveConstants() {

        Pigeon2Configuration pigeon2Configuration = new Pigeon2Configuration();
       // pigeon2Configuration.GyroTrim.GyroScalarY = GYRO_ERROR;

        swerveDrivetrainConstants = new SwerveDrivetrainConstants()
                .withCANBusName("CANivore")
                .withPigeon2Id(GYRO.getDeviceNumber())
                .withPigeon2Configs(pigeon2Configuration);

        moduleConstants = new SwerveModuleConstants[4];
        moduleConstants[0] = new SwerveModuleConstants<
                        TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
                .withDriveMotorId(FRONT_LEFT_DRIVE_MOTOR.getDeviceNumber())
                .withSteerMotorId(FRONT_LEFT_STEER_MOTOR.getDeviceNumber())
                .withEncoderId(FRONT_LEFT_STEER_ENCODER.getDeviceNumber())
                .withDriveMotorGearRatio(DRIVE_GEAR_RATIO)
                .withSteerMotorGearRatio(MK4n_STEER_GEAR_RATIO)
                .withCouplingGearRatio(COUPLING_GEAR_RATIO)
                .withDriveMotorInverted(true)
                .withSteerMotorInverted(true)
                .withEncoderInverted(false)
                .withEncoderOffset(FRONT_LEFT_STEER_OFFSET_ROTATIONS)
                .withLocationX(WHEELBASE_LENGTH_METERS / 2)
                .withLocationY(WHEEL_TRACK_WIDTH_METERS / 2)
                .withDriveMotorClosedLoopOutput(SwerveModuleConstants.ClosedLoopOutputType.Voltage)
                .withSteerMotorClosedLoopOutput(SwerveModuleConstants.ClosedLoopOutputType.Voltage)
                .withDriveMotorGains(new ConfigureSlot0Gains(0, 0, 0, 0.11464878310546875, 0))
                .withSteerMotorGains(new ConfigureSlot0Gains(100.0, 0.0, 0.0, 0.0, 0.0))
                .withDriveMotorType(SwerveModuleConstants.DriveMotorArrangement.TalonFX_Integrated)
                .withSteerMotorType(SwerveModuleConstants.SteerMotorArrangement.TalonFX_Integrated)
                .withDriveMotorInitialConfigs(new TalonFXConfiguration())
                .withSteerMotorInitialConfigs(new TalonFXConfiguration())
                .withEncoderInitialConfigs(new CANcoderConfiguration())
                .withDriveFrictionVoltage(0.25)
                .withSteerFrictionVoltage(0.001)
                .withDriveInertia(0.001)
                .withSteerInertia(0.00001)
                .withSlipCurrent(120) // TODO MEASURE
                .withFeedbackSource(SwerveModuleConstants.SteerFeedbackType.RemoteCANcoder)
                .withSpeedAt12Volts(MAX_SPEED_METERS_PER_SECOND)
                .withWheelRadius(WHEEL_RADIUS_METERS);

        moduleConstants[1] = new SwerveModuleConstants<
                        TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
                .withDriveMotorId(FRONT_RIGHT_DRIVE_MOTOR.getDeviceNumber())
                .withSteerMotorId(FRONT_RIGHT_STEER_MOTOR.getDeviceNumber())
                .withEncoderId(FRONT_RIGHT_STEER_ENCODER.getDeviceNumber())
                .withDriveMotorGearRatio(DRIVE_GEAR_RATIO)
                .withSteerMotorGearRatio(MK4n_STEER_GEAR_RATIO)
                .withCouplingGearRatio(COUPLING_GEAR_RATIO)
                .withDriveMotorInverted(true)
                .withSteerMotorInverted(true)
                .withEncoderInverted(false)
                .withEncoderOffset(FRONT_RIGHT_STEER_OFFSET_ROTATIONS)
                .withLocationX(WHEELBASE_LENGTH_METERS / 2)
                .withLocationY(-WHEEL_TRACK_WIDTH_METERS / 2)
                .withDriveMotorClosedLoopOutput(SwerveModuleConstants.ClosedLoopOutputType.Voltage)
                .withSteerMotorClosedLoopOutput(SwerveModuleConstants.ClosedLoopOutputType.Voltage)
                .withDriveMotorGains(new ConfigureSlot0Gains(0, 0, 0, 0.11464878310546875, 0))
                .withSteerMotorGains(new ConfigureSlot0Gains(100.0, 0.0, 0.0, 0.0, 0.0))
                .withDriveMotorType(SwerveModuleConstants.DriveMotorArrangement.TalonFX_Integrated)
                .withSteerMotorType(SwerveModuleConstants.SteerMotorArrangement.TalonFX_Integrated)
                .withDriveMotorInitialConfigs(new TalonFXConfiguration())
                .withSteerMotorInitialConfigs(new TalonFXConfiguration())
                .withEncoderInitialConfigs(new CANcoderConfiguration())
                .withDriveFrictionVoltage(0.25)
                .withSteerFrictionVoltage(0.001)
                .withDriveInertia(0.001)
                .withSteerInertia(0.00001)
                .withSlipCurrent(120) // TODO MEASURE
                .withFeedbackSource(SwerveModuleConstants.SteerFeedbackType.RemoteCANcoder)
                .withSpeedAt12Volts(MAX_SPEED_METERS_PER_SECOND)
                .withWheelRadius(WHEEL_RADIUS_METERS);

        moduleConstants[2] = new SwerveModuleConstants<
                        TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
                .withDriveMotorId(BACK_LEFT_DRIVE_MOTOR.getDeviceNumber())
                .withSteerMotorId(BACK_LEFT_STEER_MOTOR.getDeviceNumber())
                .withEncoderId(BACK_LEFT_STEER_ENCODER.getDeviceNumber())
                .withDriveMotorGearRatio(DRIVE_GEAR_RATIO)
                .withSteerMotorGearRatio(MK4n_STEER_GEAR_RATIO)
                .withCouplingGearRatio(COUPLING_GEAR_RATIO)
                .withDriveMotorInverted(false)
                .withSteerMotorInverted(true)
                .withEncoderInverted(false)
                .withEncoderOffset(BACK_LEFT_STEER_OFFSET_ROTATIONS)
                .withLocationX(-WHEELBASE_LENGTH_METERS / 2)
                .withLocationY(WHEEL_TRACK_WIDTH_METERS / 2)
                .withDriveMotorClosedLoopOutput(SwerveModuleConstants.ClosedLoopOutputType.Voltage)
                .withSteerMotorClosedLoopOutput(SwerveModuleConstants.ClosedLoopOutputType.Voltage)
                .withDriveMotorGains(new ConfigureSlot0Gains(0, 0, 0, 0.11464878310546875, 0))
                .withSteerMotorGains(new ConfigureSlot0Gains(100.0, 0.0, 0.0, 0.0, 0.0))
                .withDriveMotorType(SwerveModuleConstants.DriveMotorArrangement.TalonFX_Integrated)
                .withSteerMotorType(SwerveModuleConstants.SteerMotorArrangement.TalonFX_Integrated)
                .withDriveMotorInitialConfigs(new TalonFXConfiguration())
                .withSteerMotorInitialConfigs(new TalonFXConfiguration())
                .withEncoderInitialConfigs(new CANcoderConfiguration())
                .withDriveFrictionVoltage(0.25)
                .withSteerFrictionVoltage(0.001)
                .withDriveInertia(0.001)
                .withSteerInertia(0.00001)
                .withSlipCurrent(120) // TODO MEASURE
                .withFeedbackSource(SwerveModuleConstants.SteerFeedbackType.RemoteCANcoder)
                .withSpeedAt12Volts(MAX_SPEED_METERS_PER_SECOND)
                .withWheelRadius(WHEEL_RADIUS_METERS);

        moduleConstants[3] = new SwerveModuleConstants<
                        TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
                .withDriveMotorId(BACK_RIGHT_DRIVE_MOTOR.getDeviceNumber())
                .withSteerMotorId(BACK_RIGHT_STEER_MOTOR.getDeviceNumber())
                .withEncoderId(BACK_RIGHT_STEER_ENCODER.getDeviceNumber())
                .withDriveMotorGearRatio(DRIVE_GEAR_RATIO)
                .withSteerMotorGearRatio(MK4n_STEER_GEAR_RATIO)
                .withCouplingGearRatio(COUPLING_GEAR_RATIO)
                .withDriveMotorInverted(false)
                .withSteerMotorInverted(true)
                .withEncoderInverted(false)
                .withEncoderOffset(BACK_RIGHT_STEER_OFFSET_ROTATIONS)
                .withLocationX(-WHEELBASE_LENGTH_METERS / 2)
                .withLocationY(-WHEEL_TRACK_WIDTH_METERS / 2)
                .withDriveMotorClosedLoopOutput(SwerveModuleConstants.ClosedLoopOutputType.Voltage)
                .withSteerMotorClosedLoopOutput(SwerveModuleConstants.ClosedLoopOutputType.Voltage)
                .withDriveMotorGains(new ConfigureSlot0Gains(0, 0, 0, 0.11464878310546875, 0))
                .withSteerMotorGains(new ConfigureSlot0Gains(100.0, 0.0, 0.0, 0.0, 0.0))
                .withDriveMotorType(SwerveModuleConstants.DriveMotorArrangement.TalonFX_Integrated)
                .withSteerMotorType(SwerveModuleConstants.SteerMotorArrangement.TalonFX_Integrated)
                .withDriveMotorInitialConfigs(new TalonFXConfiguration())
                .withSteerMotorInitialConfigs(new TalonFXConfiguration())
                .withEncoderInitialConfigs(new CANcoderConfiguration())
                .withDriveFrictionVoltage(0.25)
                .withSteerFrictionVoltage(0.001)
                .withDriveInertia(0.001)
                .withSteerInertia(0.00001)
                .withSlipCurrent(120) // TODO MEASURE
                .withFeedbackSource(SwerveModuleConstants.SteerFeedbackType.RemoteCANcoder)
                .withSpeedAt12Volts(MAX_SPEED_METERS_PER_SECOND)
                .withWheelRadius(WHEEL_RADIUS_METERS);
    }

    public static SwerveDrivetrainConstants getSwerveDrivetrainConstants() {
        return swerveDrivetrainConstants;
    }

    public static SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>[]
            getModuleConstants() {
        return moduleConstants;
    }
}