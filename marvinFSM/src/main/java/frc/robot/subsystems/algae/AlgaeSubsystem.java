package frc.robot.subsystems.algae;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.SubsystemDataProcessor;

public class AlgaeSubsystem extends SubsystemBase {
    private final GrabberIO grabberIO;
    private final GrabberIOInputsAutoLogged grabberInputs = new GrabberIOInputsAutoLogged();

    private double algaeSpittingTimestamp = Double.NaN;
    private static final double ALGAE_SPIT_DURATION = 2; // spit for 2 seconds 
    public enum WantedState {
        IDLE,
        INTAKE,
        SPIT,
        HOLD
    }

    private enum SystemState {
        IDLING,
        INTAKING,
        SPITTING,
        HOLDING
    }
    
    private WantedState wantedState = WantedState.IDLE;
    private SystemState systemState = SystemState.IDLING;

    public AlgaeSubsystem(GrabberIO grabIO) {
        grabberIO = grabIO;
        SubsystemDataProcessor.createAndStartSubsystemDataProcessor(
            () -> {
                synchronized (grabberInputs) {
                    grabberIO.updateInputs(grabberInputs);
                }
            }, grabIO);
    }

    @Override
    public void periodic() {
        synchronized (grabberInputs) {
            Logger.processInputs("Subsystems/Algae", grabberInputs);

            systemState = handleStateTransition();
            Logger.recordOutput("Subsystems/Algae/WantedState", wantedState);
            Logger.recordOutput("Subsystems/Algae/SystemState", systemState);

            applyStates();

        }
    }

    private SystemState handleStateTransition() {
        switch (wantedState) {
            case IDLE -> {
                return SystemState.IDLING;
            }
        case SPIT -> {
            if (!DriverStation.isDisabled()) {
                if (Double.isNaN(algaeSpittingTimestamp)) {
                    algaeSpittingTimestamp = Timer.getFPGATimestamp();
                    return SystemState.SPITTING;
            } else if ((Timer.getFPGATimestamp() - algaeSpittingTimestamp) > ALGAE_SPIT_DURATION) {
                algaeSpittingTimestamp = Double.NaN;
            setWantedState(WantedState.IDLE);
            return SystemState.IDLING;
            } else {
                return SystemState.SPITTING; // <-- stay in SPITTING until duration passes
            }
            } else {
                algaeSpittingTimestamp = Double.NaN; // optional: reset if disabled
                return SystemState.IDLING;
            }
            }
            case INTAKE -> {
                return SystemState.INTAKING;
            }
            case HOLD -> {
                return SystemState.HOLDING;
            } default -> {
                return SystemState.IDLING;
            }
        }
    }

    public void applyStates() {
        switch (systemState) {
            case IDLING -> {
                grabberIO.stop();
                break;
            }
            case SPITTING -> {
                grabberIO.run(-1.0); // Spit algae
                break;
            }
            case INTAKING -> {
                grabberIO.run(1.0);
                break;
            }
            case HOLDING -> {
                grabberIO.run(0.5);
                break;
            }
        }
    }

    public void setWantedState(WantedState wantedState) {
        this.wantedState = wantedState;
    }

    public boolean isHolding() {
        return (systemState == SystemState.HOLDING);
    }
}
