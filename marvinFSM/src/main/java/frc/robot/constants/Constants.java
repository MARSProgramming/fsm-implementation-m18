package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;

import java.util.Map;

public final class Constants {
    public static final class SuperstructureConstants {
        public static final double X_OFFSET_RIGHT_L1_INCHES = 18.503; // actually need to determine this on the field
        public static final double Y_OFFSET_RIGHT_L1_INCHES = 16.535; // actually need to determine this on the field

        public static final double X_OFFSET_RIGHT_L2_INCHES = 18.503; // actually need to determine this on the field
        public static final double Y_OFFSET_RIGHT_L2_INCHES = 16.535; // actually need to determine this on the field

        public static final double X_OFFSET_RIGHT_L3_INCHES = 18.503; // actually need to determine this on the field
        public static final double Y_OFFSET_RIGHT_L3_INCHES = 16.535; // actually need to determine this on the field

        public static final double X_OFFSET_RIGHT_L4_INCHES = 17.913; // actually need to determine this on the field
        public static final double Y_OFFSET_RIGHT_L4_INCHES = 16.929; // actually need to determine this on the field

        public static final double X_OFFSET_LEFT_L1_INCHES = 18.503; // actually need to determine this on the field
        public static final double Y_OFFSET_LEFT_L1_INCHES = 1.1811; // actually need to determine this on the field

        public static final double X_OFFSET_LEFT_L2_INCHES = 18.503; // actually need to determine this on the field
        public static final double Y_OFFSET_LEFT_L2_INCHES = 1.1811; // actually need to determine this on the field

        public static final double X_OFFSET_LEFT_L3_INCHES = 18.503; // actually need to determine this on the field
        public static final double Y_OFFSET_LEFT_L3_INCHES = 1.1811; // actually need to determine this on the field

        public static final double X_OFFSET_LEFT_L4_INCHES = 17.322; // actually need to determine this on the field
        public static final double Y_OFFSET_LEFT_L4_INCHES = 1.9685; // actually need to determine this on the field

        public static final double X_OFFSET_FROM_TAG_FOR_INTAKING_ALGAE_INCHES = 18.0; // this is from the reef
        // rotation would be 0 for reef algae pickup

       // public static final double X_OFFSET_FROM_TAG_FOR_INTERMEDIATE_INTAKING_ALGAE_INCHES = 30.0;
       // public static final double X_OFFSET_FROM_TAG_FOR_BACKOUT_INTAKING_ALGAE_INCHES = 50.0;
       // public static final double X_OFFSET_FROM_TAG_FOR_L1_BACKOUT_INCHES = 10.0;

      //  public static final double Y_OFFSET_FROM_TAG_FOR_SCORING_ON_REEF_INCHES = 6.5;
      //  public static final double Y_OFFSET_FROM_TAG_FOR_SCORING_L1_INCHES = 9.0;



        public enum ScoringLevel {
            L4,
            L3,
            L2,
            L1
        }

        public enum ScoringSide {
            RIGHT,
            LEFT
        }

        public enum AutomationLevel {
            AUTO_RELEASE,
            AUTO_DRIVE_AND_MANUAL_RELEASE,
            NO_AUTO_DRIVE
        }

        public enum ReefSelectionMethod {
            POSE,
            ROTATION
        }
    }

    public static final class ReefConstants {
        public enum ReefFaces {
            AB,
            CD,
            EF,
            GH,
            IJ,
            KL
        }

        public enum AlgaeIntakeLocation {
            L2,
            L3
        }

        public static final class AlgaeIntakeMapping {
            public final AlgaeIntakeLocation FRONT;
            public final AlgaeIntakeLocation BACK;

            public AlgaeIntakeMapping(AlgaeIntakeLocation front, AlgaeIntakeLocation back) {
                FRONT = front;
                BACK = back;
            }
        }

        public static final class ScoringCoralMappingRotationToTagID {
            public final int FRONT_ID;
            public final int BACK_ID;

            public ScoringCoralMappingRotationToTagID(int frontID, int backID) {
                FRONT_ID = frontID;
                BACK_ID = backID;
            }
        }

        public static final Map<Pose2d, Integer> blueAlliancePoseToTagIDsMap = Map.of(
                Field.getTagPose(21).toPose2d(), 21,
                Field.getTagPose(20).toPose2d(), 20,
                Field.getTagPose(19).toPose2d(), 19,
                Field.getTagPose(18).toPose2d(), 18,
                Field.getTagPose(17).toPose2d(), 17,
                Field.getTagPose(22).toPose2d(), 22);

        public static final Map<Pose2d, Integer> redAlliancePoseToTagIDsMap = Map.of(
                Field.getTagPose(6).toPose2d(), 6,
                Field.getTagPose(7).toPose2d(), 7,
                Field.getTagPose(8).toPose2d(), 8,
                Field.getTagPose(9).toPose2d(), 9,
                Field.getTagPose(10).toPose2d(), 10,
                Field.getTagPose(11).toPose2d(), 11);

        public static final Map<Rotation2d, ScoringCoralMappingRotationToTagID> redAllianceAngleToTagIDsMap = Map.of(
                Rotation2d.fromDegrees(-60),
                new ScoringCoralMappingRotationToTagID(9, 6),
                Rotation2d.fromDegrees(-120),
                new ScoringCoralMappingRotationToTagID(8, 11),
                Rotation2d.k180deg,
                new ScoringCoralMappingRotationToTagID(7, 10),
                Rotation2d.fromDegrees(120),
                new ScoringCoralMappingRotationToTagID(6, 9),
                Rotation2d.fromDegrees(60),
                new ScoringCoralMappingRotationToTagID(11, 8),
                Rotation2d.kZero,
                new ScoringCoralMappingRotationToTagID(10, 7));

        public static final Map<Rotation2d, ScoringCoralMappingRotationToTagID> blueAllianceAngleToTagIDsMap = Map.of(
                Rotation2d.fromDegrees(-60),
                new ScoringCoralMappingRotationToTagID(19, 22),
                Rotation2d.fromDegrees(-120),
                new ScoringCoralMappingRotationToTagID(20, 17),
                Rotation2d.k180deg,
                new ScoringCoralMappingRotationToTagID(21, 18),
                Rotation2d.fromDegrees(120),
                new ScoringCoralMappingRotationToTagID(22, 19),
                Rotation2d.fromDegrees(60),
                new ScoringCoralMappingRotationToTagID(17, 20),
                Rotation2d.kZero,
                new ScoringCoralMappingRotationToTagID(18, 21));

        public static final Map<Rotation2d, AlgaeIntakeMapping> redAllianceAlgae = Map.of(
                Rotation2d.fromDegrees(0),
                new AlgaeIntakeMapping(AlgaeIntakeLocation.L2, AlgaeIntakeLocation.L3),
                Rotation2d.fromDegrees(60),
                new AlgaeIntakeMapping(AlgaeIntakeLocation.L3, AlgaeIntakeLocation.L2),
                Rotation2d.fromDegrees(120),
                new AlgaeIntakeMapping(AlgaeIntakeLocation.L2, AlgaeIntakeLocation.L3),
                Rotation2d.fromDegrees(180),
                new AlgaeIntakeMapping(AlgaeIntakeLocation.L3, AlgaeIntakeLocation.L2),
                Rotation2d.fromDegrees(-120),
                new AlgaeIntakeMapping(AlgaeIntakeLocation.L2, AlgaeIntakeLocation.L3),
                Rotation2d.fromDegrees(-60),
                new AlgaeIntakeMapping(AlgaeIntakeLocation.L3, AlgaeIntakeLocation.L2));

        public static final Map<Rotation2d, AlgaeIntakeMapping> blueAllianceAlgae = Map.of(
                Rotation2d.fromDegrees(0),
                new AlgaeIntakeMapping(AlgaeIntakeLocation.L3, AlgaeIntakeLocation.L2),
                Rotation2d.fromDegrees(60),
                new AlgaeIntakeMapping(AlgaeIntakeLocation.L2, AlgaeIntakeLocation.L3),
                Rotation2d.fromDegrees(120),
                new AlgaeIntakeMapping(AlgaeIntakeLocation.L3, AlgaeIntakeLocation.L2),
                Rotation2d.fromDegrees(180),
                new AlgaeIntakeMapping(AlgaeIntakeLocation.L2, AlgaeIntakeLocation.L3),
                Rotation2d.fromDegrees(-120),
                new AlgaeIntakeMapping(AlgaeIntakeLocation.L3, AlgaeIntakeLocation.L2),
                Rotation2d.fromDegrees(-60),
                new AlgaeIntakeMapping(AlgaeIntakeLocation.L2, AlgaeIntakeLocation.L3));

        public static final Map<ReefFaces, Integer> redAllianceReefFacesToIds = Map.of(
                ReefFaces.AB, 7,
                ReefFaces.CD, 8,
                ReefFaces.EF, 9,
                ReefFaces.GH, 10,
                ReefFaces.IJ, 11,
                ReefFaces.KL, 6);

        public static final Map<ReefFaces, Integer> blueAllianceReefFacesToIds = Map.of(
                ReefFaces.AB, 18,
                ReefFaces.CD, 17,
                ReefFaces.EF, 22,
                ReefFaces.GH, 21,
                ReefFaces.IJ, 20,
                ReefFaces.KL, 19);
    }

    public static final class ElevatorConstants {
            public static double ELEVATOR_MAX = 8.3;
            public static  double ELEVATOR_L4 = 8.25;
            public static double ELEVATOR_L3 = 4.5;
            public static double ELEVATOR_L2 = 2.08;
            public static double ELEVATOR_L1 = 0;
            public static double ELEVATOR_ALGAE_TOP = 7.7;
            public static double ELEVATOR_ALGAE_BOT = 4.9;
            public static double ELEVATOR_ALGAE_TEE = 2.2;
            public static double ELEVATOR_ALGAE_GROUND = 0.72;
            public static double ELEVATOR_CLIMB = 0;
            public static double ELEVATOR_CLIMBING_VOLTAGE = -5;
            public static double ELEVATOR_ZEROING_VOLTAGE = -6;

            public static double ELEVATOR_TEST_POSITION = 1;            
    }

  
    public static final class IntakeConstants {
        public static final class CollectingVoltages {}

        public static final class EjectingVoltages {}

        public static final class CoralGunConstants {
            public static final double HOLDING_CORAL_PASSIVE_PERCENT = -0.2;
            public static final double IR_SENSOR_THRESHOLD = 1.8;
        }
    }

    public static final boolean SILENCE_JOYSTICK_WARNINGS_IN_SIMULATOR = true;
    public static final Mode currentMode = Mode.REAL;
    public static final String operatorDashboardName = "Dashboard";
    public static final String autoChooserName = "SmartDashboard/Auto/Programs";

    public static final class SysIdConstants {
        public static final Velocity<VoltageUnit> TRANSLATION_RAMP_RATE = null;
        public static final Voltage TRANSLATION_STEP_RATE = Units.Volts.of(7);
        public static final Time TRANSLATION_TIMEOUT = Units.Seconds.of(5);

        /* This is in radians per second², but SysId only supports "volts per second" */
        public static final Velocity<VoltageUnit> ROTATION_RAMP_RATE =
                Units.Volts.of(Math.PI / 6).per(Units.Second);
        /* This is in radians per second, but SysId only supports "volts" */
        public static final Voltage ROTATION_STEP_RATE = Units.Volts.of(Math.PI);
        public static final Time ROTATION_TIMEOUT = Units.Seconds.of(5);

        public static final Velocity<VoltageUnit> STEER_RAMP_RATE = null;
        public static final Voltage STEER_STEP_RATE = Units.Volts.of(7);
        public static final Time STEER_TIMEOUT = null;
    }

    public static enum Mode {
        /**
         * Running on a real robot.
         */
        REAL,

        /**
         * Running a physics simulator.
         */
        SIM,

        /**
         * Replaying from a log file.
         */
        REPLAY
    }
}