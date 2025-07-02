package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.constants.Constants;
import frc.robot.constants.Field;


import java.util.ArrayList;
import java.util.List;

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
}