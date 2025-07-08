package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.RobotState;
import frc.robot.util.SubsystemDataProcessor;
import org.littletonrobotics.junction.AutoLog;


public interface SwerveIO extends SubsystemDataProcessor.IODataRefresher {
    @AutoLog
    class SwerveIOInputs {
        public Pose2d Pose = new Pose2d();
        public ChassisSpeeds Speeds = new ChassisSpeeds();
        public SwerveModuleState[] ModuleStates;
        public SwerveModuleState[] ModuleTargets;
        public SwerveModulePosition[] ModulePositions;
        public Rotation2d RawHeading = new Rotation2d();
        public double Timestamp;
        public double OdometryPeriod;
        public int SuccessfulDaqs;
        public int FailedDaqs;

        void logState(SwerveDrivetrain.SwerveDriveState state) {
            this.Pose = state.Pose;
            this.RawHeading = state.RawHeading;
            this.ModuleStates = state.ModuleStates;
            this.ModuleTargets = state.ModuleTargets;
            this.ModulePositions = state.ModulePositions;
            this.Speeds = state.Speeds;
            this.SuccessfulDaqs = state.SuccessfulDaqs;
            this.FailedDaqs = state.FailedDaqs;
            this.OdometryPeriod = state.OdometryPeriod;
            this.Timestamp = state.OdometryPeriod;
            RobotState.getInstance().addPoseObservation(new RobotState.SwerveDriveObservation(this.Pose, this.Speeds));
        }
    }

    @AutoLog
    class ModuleIOInputs {
        public double driveSupplyCurrentAmps = 0.0;
        public double driveStatorCurrentAmps = 0.0;
        public double driveAppliedVolts = 0.0;
        public double driveTemperature = 0.0;

        public double steerSupplyCurrentAmps = 0.0;
        public double steerStatorCurrentAmps = 0.0;
        public double steerAppliedVolts = 0.0;
        public double steerTemperature = 0.0;
    }

    default void updateSwerveInputs(SwerveIOInputs inputs) {}

    default void updateModuleInputs(ModuleIOInputs... inputs) {}

    default void registerTelemetryFunction(SwerveIOInputs inputs) {}

    default void setSwerveState(SwerveRequest request) {}

    default void resetRotation() {}

    default void getEstimatedPosition() {}

    default void resetToParamaterizedRotation(Rotation2d rotation2d) {}

    default void updateSimState() {}

    default void addMeasure(Pose2d poseToUpdate, double timestampSeconds, Matrix<N3,N1> visionMeasurementStdDevs) {}

    default void resetRobotTranslation(Translation2d translation2d) {}

    @Override
    default void refreshData() {}

}

