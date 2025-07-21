package frc.robot.auto;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.SuperstructureConstants;
import frc.robot.constants.Field;
import frc.robot.subsystems.Superstructure;

public class AutoFactory {
    private final DriverStation.Alliance alliance;
    private final RobotContainer robotContainer;
    private final Choreo.TrajectoryCache trajectoryCache;

    private final SuperstructureConstants.ScoringLevel intermediateLevel = SuperstructureConstants.ScoringLevel.L2;
    
    AutoFactory(final DriverStation.Alliance alliance, final RobotContainer robotContainer) {
        this.alliance = alliance;
        this.robotContainer = robotContainer;

        trajectoryCache = new Choreo.TrajectoryCache();
    }

    private static final Command IDLE = Commands.idle();

    Pair <Pose2d, Command> createIdleCommand() {
        return Pair.of(Field.getFarLeftStartingPose(alliance), IDLE);
    }


    // Auto Factory handlers

    private String trajectoryName(final Location start, final Location end) {
        var name = "%S_TO_%S".formatted(start, end);
        var x = "%S_%S".formatted(alliance, name);
        System.out.println("%S_%S".formatted(alliance, name));
        return x;
    }

    Command setState(Superstructure.WantedSuperState state) {
        return robotContainer.getSuperstructure().setStateCommand(state);
    }

    Command followTrajectory(Trajectory<SwerveSample> trajectory) {
        return new InstantCommand(() -> robotContainer.getSwerveSubsystem().setDesiredChoreoTrajectory(trajectory));
    }

    Command driveToPoint(Pose2d point, double maxVelocityOutputForDriveToPoint) {
        return new InstantCommand(() -> 
        robotContainer.getSwerveSubsystem().
        setDesiredPoseForDriveToPointWithConstraints(point, maxVelocityOutputForDriveToPoint, 1.0))
        .andThen(
            new WaitUntilCommand(
                () -> robotContainer.getSwerveSubsystem()
                .isAtDriveToPointSetpoint()));
    }

    Command driveToPointWithUnconstrainedMaxVelocity(Pose2d point, double maxVelocityOutputForDriveToPoint) {
        return new InstantCommand(() -> robotContainer
                        .getSwerveSubsystem()
                        .setDesiredPoseForDriveToPointWithConstraints(
                                point, maxVelocityOutputForDriveToPoint, Double.NaN))
                .andThen(new WaitUntilCommand(
                        () -> robotContainer.getSwerveSubsystem().isAtDriveToPointSetpoint()));
    } 

    public Pose2d getAutoScoringPose(
        Constants.ReefConstants.ReefFaces reefFaces, Superstructure.WantedSuperState superState) {
    var map = alliance == DriverStation.Alliance.Blue
            ? Constants.ReefConstants.blueAllianceReefFacesToIds
            : Constants.ReefConstants.redAllianceReefFacesToIds;
    var id = map.get(reefFaces);
    var scoringSide = (superState == Superstructure.WantedSuperState.SCORE_LEFT_L4)
            ? Constants.SuperstructureConstants.ScoringSide.LEFT
            : Constants.SuperstructureConstants.ScoringSide.RIGHT;
    return Field.getDesiredFinalScoringPoseForCoral(
            id, scoringSide, SuperstructureConstants.ScoringLevel.L4);
}
}
