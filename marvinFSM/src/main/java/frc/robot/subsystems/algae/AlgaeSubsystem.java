package frc.robot.subsystems.algae;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.SubsystemDataProcessor;

public class AlgaeSubsystem extends SubsystemBase {
    private final GrabberIO grabberIO;
    private final GrabberIOInputsAutoLogged grabberInputs = new GrabberIOInputsAutoLogged();

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
                return SystemState.SPITTING;
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
            }
            case HOLDING -> {
                grabberIO.run(0.5);
            }
        }
    }
}
