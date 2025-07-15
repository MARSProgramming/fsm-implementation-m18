package frc.robot.subsystems;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.Constants;
import frc.robot.subsystems.algae.AlgaeSubsystem;
import frc.robot.subsystems.coral.CoralSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.util.Dash;

public class Superstructure extends SubsystemBase {
    private final SwerveSubsystem swerveSubsystem;
    private final CoralSubsystem coralSubsystem;
    private final AlgaeSubsystem algaeSubsystem;
    private final LEDSubsystem ledSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;


    private final Dash operatorDashboard;
    private final CommandXboxController controller = new CommandXboxController(0);

    public enum WantedSuperState {
        HOME,
        STOPPED,
        DEFAULT_STATE,
        FORCE_RELOCALIZE_LEFT,
        FORCE_RELOCALIZE_RIGHT,
        SCORE_L1_MANUAL_ALIGN,
        SCORE_L1_LEFT_BASE,
        SCORE_L1_RIGHT_BASE,
        SCORE_L1_LEFT_TOP,
        SCORE_L1_RIGHT_TOP,
        SCORE_LEFT_L2,
        SCORE_LEFT_L3,
        SCORE_LEFT_L4,
        SCORE_RIGHT_L2,
        SCORE_RIGHT_L3,
        SCORE_RIGHT_L4,
        MANUAL_L4,
        MANUAL_L3,
        MANUAL_L2,
        MANUAL_L1,
        INTAKE_ALGAE_FROM_REEF_TOP,
        INTAKE_ALGAE_FROM_REEF_BOT,
        MOVE_ALGAE_TO_PROCESSOR_POSITION,
        SCORE_ALGAE_IN_PROCESSOR,
        CLIMB
    }

    public enum CurrentSuperState {
        HOME,
        STOPPED,
        HOLDING_CORAL_TELEOP,
        NO_PIECE_AUTO,
        HOLDING_CORAL_AUTO,
        HOLDING_ALGAE,
        FORCE_RELOCALIZE_LEFT,
        FORCE_RELOCALIZE_RIGHT,
        SCORE_TELEOP_L1_MANUAL_ALIGNMENT,
        SCORE_LEFT_TELEOP_L2,
        SCORE_LEFT_TELEOP_L3,
        SCORE_LEFT_TELEOP_L4,
        SCORE_RIGHT_TELEOP_L2,
        SCORE_RIGHT_TELEOP_L3,
        SCORE_RIGHT_TELEOP_L4,
        SCORE_AUTO_L1,
        SCORE_LEFT_AUTO_L2,
        SCORE_LEFT_AUTO_L3,
        SCORE_LEFT_AUTO_L4,
        SCORE_RIGHT_AUTO_L2,
        SCORE_RIGHT_AUTO_L3,
        SCORE_RIGHT_AUTO_L4,
        MANUAL_L4,
        MANUAL_L3,
        MANUAL_L2,
        MANUAL_L1,
        INTAKE_ALGAE_FROM_HP,
        INTAKE_ALGAE_FROM_REEF_TOP,
        INTAKE_ALGAE_FROM_REEF_BOT,
        MOVE_ALGAE_TO_PROCESSOR_POSITION,
        SCORE_ALGAE_IN_PROCESSOR,
        CLIMB
    }

    private WantedSuperState wantedSuperState = WantedSuperState.STOPPED;
    private CurrentSuperState currentSuperState = CurrentSuperState.STOPPED;
    private CurrentSuperState previousSuperState;

    private boolean hasDriveToPointSetPointBeenSet = false;

    private NeutralModeValue pastSwitchValue = NeutralModeValue.Brake;

    private boolean hasPoseBeenResetPrematch = false;

    private boolean allowExternalCommandToAccessLEDS = false;

    private boolean hasElevatorReachedSetpoint = false;
    private boolean hasCoral = false;
    private boolean algaeMotorStalling = false;    

    public Superstructure(SwerveSubsystem swerve, CoralSubsystem coral, AlgaeSubsystem algae, ElevatorSubsystem elev, LEDSubsystem led, Dash dash) {
        swerveSubsystem = swerve;
        coralSubsystem = coral;
        algaeSubsystem = algae;
        elevatorSubsystem = elev;
        ledSubsystem = led;
        operatorDashboard = dash;
    }



    // state machine methods. the following methods are designed to handle all state machine logic.

    private void home() {
        ledSubsystem.setWantedAction(LEDSubsystem.WantedState.DISPLAY_ROBOT_ZERO_ACTION);

        if (elevatorSubsystem.hasHomeCompleted() && previousSuperState == CurrentSuperState.HOME) {
            elevatorSubsystem.setWantedState(ElevatorSubsystem.WantedState.IDLE);
        } else {
            elevatorSubsystem.setWantedState(ElevatorSubsystem.WantedState.HOME);
        }

        if (elevatorSubsystem.hasHomeCompleted() && previousSuperState == currentSuperState.HOME) {
            setWantedSuperState(WantedSuperState.DEFAULT_STATE);
        }
     }

     private void stopped() {
        elevatorSubsystem.setWantedState(ElevatorSubsystem.WantedState.IDLE);
        coralSubsystem.setWantedState(CoralSubsystem.WantedState.IDLE);
        algaeSubsystem.setWantedState(AlgaeSubsystem.WantedState.IDLE);
        ledSubsystem.setWantedAction(elevatorSubsystem.hasHomeCompleted() ? LEDSubsystem.WantedState.DISPLAY_ROBOT_ELEVATOR_ZEROED : LEDSubsystem.WantedState.DISPLAY_ROBOT_ELEVATOR_NOT_ZEROED);
     }

     private void holdingAlgae() {
        hasDriveToPointSetPointBeenSet = false;
        elevatorSubsystem.setDesiredElevatorSetpoint(Constants.ElevatorConstants.ELEVATOR_ALGAE_TEE);
        elevatorSubsystem.setWantedState(ElevatorSubsystem.WantedState.MOVE_TO_POSITION);

        swerveSubsystem.setWantedState(SwerveSubsystem.WantedState.TELEOP_DRIVE);
        swerveSubsystem.setTeleopVelocityCoefficient(1.0);
        swerveSubsystem.setRotationVelocityCoefficient(1.0);

        ledSubsystem.setWantedAction(LEDSubsystem.WantedState.DISPLAY_HOLDING_ALGAE);
        
        algaeSubsystem.setWantedState(AlgaeSubsystem.WantedState.HOLD);
     }

     private void holdingCoral() {
        hasDriveToPointSetPointBeenSet = false;

        elevatorSubsystem.setDesiredElevatorSetpoint(Constants.ElevatorConstants.ELEVATOR_L1);
        elevatorSubsystem.setWantedState(ElevatorSubsystem.WantedState.MOVE_TO_POSITION);

        coralSubsystem.setWantedState(CoralSubsystem.WantedState.PASSIVE_INTAKE);

        swerveSubsystem.setWantedState(SwerveSubsystem.WantedState.TELEOP_DRIVE);
        swerveSubsystem.setTeleopVelocityCoefficient(1.0);
        swerveSubsystem.setRotationVelocityCoefficient(1.0);
     }



     // Superstructure methods.

     public void setWantedSuperState(WantedSuperState superState) {
        this.wantedSuperState = superState;
     }

     public Command setStateCommand(WantedSuperState superState) {
        return setStateCommand(superState);
     }

     public Command setStateCommandWithServoCheck(WantedSuperState superState, boolean servo) {
        Command commandToReturn = new InstantCommand(() -> setWantedSuperState(superState));
        // may need logic here that checks the servo position before running the command
        return commandToReturn;
     }

     public boolean hasCoral() {
        return coralSubsystem.hasCoral();
     }

    
}
