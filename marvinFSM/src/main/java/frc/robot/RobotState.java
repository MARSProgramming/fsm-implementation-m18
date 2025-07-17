package frc.robot;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.constants.Constants;
import frc.robot.constants.Field;


import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

public class RobotState {
    private static RobotState instance;

    public static RobotState getInstance() {
        if (instance == null) {
            instance = new RobotState();
        }
        return instance;
    }

    private final List<AprilTagObservation> aprilTagObservations = new ArrayList<>();
    private Pose2d robotToFieldFromSwerveDriveOdometry = new Pose2d();
    private ChassisSpeeds robotChassisSpeeds = new ChassisSpeeds();

    public record AprilTagObservation(
            String cameraName, int tagId, Pose2d robotPoseFromCamera) {}

    public record SwerveDriveObservation(Pose2d robotPose, ChassisSpeeds robotSpeeds) {}

    public void addPoseObservation(SwerveDriveObservation observation) {
        this.robotToFieldFromSwerveDriveOdometry = observation.robotPose;
        this.robotChassisSpeeds = observation.robotSpeeds;
    }

    public void addVisionObservation(AprilTagObservation... observations) {
        aprilTagObservations.clear();
        for (var observation : observations) {
            aprilTagObservations.add(observation);
        }
    }

    public List<AprilTagObservation> getAprilTagObservations() {
        return aprilTagObservations;
    }

    public Pose2d getRobotPoseFromSwerveDriveOdometry() {
        return robotToFieldFromSwerveDriveOdometry;
    }

    public ChassisSpeeds getRobotChassisSpeeds() {
        return robotChassisSpeeds;
    }


    public int getClosestTagId() {
        var pose = RobotState.getInstance().getRobotPoseFromSwerveDriveOdometry();
        List<Pose2d> possiblePoses = List.of();
        int correctTagID = 0;

        if (Field.isBlueAlliance()) {
            possiblePoses = Constants.ReefConstants.blueAlliancePoseToTagIDsMap.keySet().stream()
                    .toList();
            correctTagID = Constants.ReefConstants.blueAlliancePoseToTagIDsMap.get(pose.nearest(possiblePoses));

        } else {
            possiblePoses = Constants.ReefConstants.redAlliancePoseToTagIDsMap.keySet().stream()
                    .toList();
            correctTagID = Constants.ReefConstants.redAlliancePoseToTagIDsMap.get(pose.nearest(possiblePoses));
        }

        return correctTagID;
    }

        public Rotation2d getClosest60Degrees() {
        double[] list = {60, 120, 180, -60, -120, 0}; // currently, our robot "front" modules are not listed correctly.
        // for a proper implementation, we will need to rename the "front". this is the most optimal solution.
        double desiredRotation = 0;
        for (double e : list) {
            var rotation = Rotation2d.fromDegrees(e);
            if (robotToFieldFromSwerveDriveOdometry
                                    .getRotation()
                                    .minus(rotation)
                                    .getDegrees()
                            < 30.0
                    && robotToFieldFromSwerveDriveOdometry
                                    .getRotation()
                                    .minus(rotation)
                                    .getDegrees()
                            >= -30) {
                desiredRotation = e;
            }
        }

        // hotfix: add 90 degrees to every rotation to accomplish goal because our robot's front is "cage" side
        Logger.recordOutput("RobotState/Closest60DegreeAngle", desiredRotation);
        return Rotation2d.fromDegrees(desiredRotation + 90);
    }

    public Constants.ReefConstants.ScoringCoralMappingRotationToTagID getValidTagIDsFromClosest60DegreeRotation() {
        return getValidTagIDsFromClosest60DegreeRotation(getClosest60Degrees());
    }

    public Constants.ReefConstants.ScoringCoralMappingRotationToTagID getValidTagIDsFromClosest60DegreeRotation(
            Rotation2d closest60Degrees) {
        var ids = Field.isBlueAlliance()
                ? Constants.ReefConstants.blueAllianceAngleToTagIDsMap.get(closest60Degrees)
                : Constants.ReefConstants.redAllianceAngleToTagIDsMap.get(closest60Degrees);
        Logger.recordOutput("RobotState/ValidTagIdsFromRotation/FrontID", ids.FRONT_ID);
        Logger.recordOutput("RobotState/ValidTagIdsFromRotation/BackID", ids.BACK_ID);
        return ids;
    }
}