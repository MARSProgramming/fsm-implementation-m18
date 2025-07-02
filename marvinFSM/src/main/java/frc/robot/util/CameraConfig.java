package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

public class CameraConfig {
    public enum Location {
        FRONT_LEFT,
        FRONT_RIGHT,
        BACK_LEFT,
        BACK_RIGHT,
        NONE
    }

    private final String name;
    public double LimelightMountingRollRadians = 0;
    public double LimelightMountingYawRadians = 0;
    public double LimelightMountingPitchRadians = 0;
    public double LimelightHeightOffsetMeters = 0;
    public double LimelightLengthOffsetMeters = 0;
    public double LimelightWidthOffsetMeters = 0;
    public Location limelightLocation = Location.NONE;

    public double LimelightDistanceScalarValue = 0;

    public CameraConfig(String name) {
        this.name = name;
    }

    public CameraConfig() {
        this("limelight");
    }

    public String getName() {
        return name;
    }

    public CameraConfig withMountingRoll(double rollRadians) {
        this.LimelightMountingRollRadians = rollRadians;
        return this;
    }

    public CameraConfig withMountingYaw(double yawRadians) {
        this.LimelightMountingYawRadians = yawRadians;
        return this;
    }

    public CameraConfig withMountingPitch(double pitchRadians) {
        this.LimelightMountingPitchRadians = pitchRadians;
        return this;
    }

    public CameraConfig withHeightOffset(double heightOffsetMeters) {
        this.LimelightHeightOffsetMeters = heightOffsetMeters;
        return this;
    }

    public CameraConfig withLengthOffset(double lengthOffsetMeters) {
        this.LimelightLengthOffsetMeters = lengthOffsetMeters;
        return this;
    }

    public CameraConfig withWidthOffset(double widthOffsetMeters) {
        this.LimelightWidthOffsetMeters = widthOffsetMeters;
        return this;
    }

    public CameraConfig withDistanceScalar(double distanceScalar) {
        this.LimelightDistanceScalarValue = distanceScalar;
        return this;
    }

    public Translation3d getTranslationOffset() {
        return new Translation3d(
                this.LimelightLengthOffsetMeters, this.LimelightHeightOffsetMeters, this.LimelightWidthOffsetMeters);
    }

    public Rotation3d getRotationOffset() {
        return new Rotation3d(
                this.LimelightMountingRollRadians,
                this.LimelightMountingPitchRadians,
                this.LimelightMountingYawRadians);
    }

    public Translation2d getTranslationToRobotCenter() {
        return new Translation2d(this.LimelightLengthOffsetMeters, this.LimelightWidthOffsetMeters);
    }

    public Location getLocation() {
        return limelightLocation;
    }
}