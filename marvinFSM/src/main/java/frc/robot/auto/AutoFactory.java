package frc.robot.auto;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.ReefConstants;
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

    Pair<Pose2d, Command> createIKLJAuto() {

        var initialPose = Field.getLeftStartingPose(alliance);

        return Pair.of(
            initialPose, 
            Commands.sequence(
                followThenScore(
                    ReefConstants.ReefFaces.IJ, 
                    Superstructure.WantedSuperState.SCORE_LEFT_L4,
                    Units.inchesToMeters(24),
                    Units.feetToMeters(8.0)
                    ),
                followThenIntakeFromStation(
                    Field.getLeftStationPickup(), 
                    Units.feetToMeters(8.0)
                    ),
                followThenScore(
                    ReefConstants.ReefFaces.KL, 
                    Superstructure.WantedSuperState.SCORE_LEFT_L4,
                    Units.inchesToMeters(24),
                    Units.feetToMeters(8.0)
                    ),
                followThenIntakeFromStation(
                    Field.getLeftStationPickup(), 
                    Units.feetToMeters(8.0)
                    ),
                followThenScore(
                    ReefConstants.ReefFaces.KL, 
                    Superstructure.WantedSuperState.SCORE_RIGHT_L4,
                    Units.inchesToMeters(24),
                    Units.feetToMeters(8.0)
                    ),
                setState(Superstructure.WantedSuperState.DEFAULT_STATE)
            ));
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

    private Command followThenScore(
        ReefConstants.ReefFaces reefFaces,
        Superstructure.WantedSuperState scoreState,
        double distanceFromEndOfPathToStartElevation,
        double maxVelocity) {
            var desiredPose = getAutoScoringPose(reefFaces, scoreState);
            return ((driveToPoint(desiredPose, maxVelocity))
            .alongWith(new WaitUntilCommand(   
                () -> robotContainer.getSwerveSubsystem().getDistanceFromDriveToPointSetpoint() < distanceFromEndOfPathToStartElevation)
                ).andThen(setState(scoreState)))
            .andThen(waitForCoralRelease().raceWith(new WaitCommand(1.0)));
    }

    private Command followThenScore(
            Constants.ReefConstants.ReefFaces reefFaces, Superstructure.WantedSuperState scoreState) {
        var desiredPose = getAutoScoringPose(reefFaces, scoreState);
        return ((driveToPoint(desiredPose, Units.feetToMeters(12.0)).alongWith(setState(scoreState))))
                .andThen(waitForCoralRelease().raceWith(new WaitCommand(1.0)));
    }

    private Command followThenScore(
            Constants.ReefConstants.ReefFaces reefFaces,
            Trajectory<SwerveSample> path,
            Superstructure.WantedSuperState scoreState,
            Superstructure.WantedSuperState noCoralState) {
        return (followTrajectory(path)
                        .andThen(new WaitUntilCommand(
                                robotContainer.getSwerveSubsystem()::isAtEndOfChoreoTrajectoryOrDriveToPoint)))
                .alongWith(new WaitUntilCommand(
                                () -> robotContainer.getSwerveSubsystem().getRobotDistanceFromChoreoEndpoint()
                                        < 2)
                        .andThen(new ConditionalCommand(
                                followThenScore(reefFaces, scoreState),
                                setState(noCoralState),
                                () -> robotContainer.getSuperstructure().hasCoral())));
    }

    private Command followThenScore(
        ReefConstants.ReefFaces reefFaces,
        Trajectory<SwerveSample> path,
        Superstructure.WantedSuperState scoreState) {
    var noCoralState = (scoreState == Superstructure.WantedSuperState.SCORE_LEFT_L4)
            ? Superstructure.WantedSuperState.FORCE_RELOCALIZE
            : Superstructure.WantedSuperState.FORCE_RELOCALIZE;
    return followThenScore(reefFaces, path, scoreState, noCoralState);
}

    private Command followThenIntakeFromStation(Pose2d intakePose, double intakeVelocity) {
        return (driveToPoint(intakePose, intakeVelocity)
                        .alongWith(setState(Superstructure.WantedSuperState.INTAKE_CORAL_FROM_STATION))
                        .andThen(Commands.waitSeconds(2.0)))
                .raceWith(waitForCoralAcquisition());
    }

    private Command waitForCoralRelease() {
        return new WaitUntilCommand(() -> !robotContainer.getSuperstructure().hasCoral());
    }

    private Command waitForCoralAcquisition() {
        return new WaitUntilCommand(() -> robotContainer.getSuperstructure().hasCoral());
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
