package frc.robot.subsystems.coralgun;

import java.nio.channels.ScatteringByteChannel;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.SubsystemDataProcessor;

public class CoralgunSubsystem extends SubsystemBase {
    private final SpitterIO spitterIO;
    private final SensorIO sensorIO;

    private final SensorIOInputsAutoLogged sensorInputs = new SensorIOInputsAutoLogged();
    private final SpitterIOInputsAutoLogged spitterInputs = new SpitterIOInputsAutoLogged();

    public boolean hasCoral = false;

    public enum WantedState {
        IDLE,
        SPIT,
        EJECT,
        SPIT_L1,
        PASSIVE_INTAKE,
    }

    private enum SystemState {
        IDLING,
        SPITTING,
        EJECTING,
        SPITTING_L1,
        PASSIVE_INTAKING
    }

    private WantedState wantedState = WantedState.IDLE;
    private SystemState systemState = SystemState.IDLING;

    public CoralgunSubsystem(SpitterIO spitIO, SensorIO sensIO) {
        spitterIO = spitIO;
        sensorIO = sensIO;

        SubsystemDataProcessor.createAndStartSubsystemDataProcessor(
            () -> {
                synchronized (sensorInputs) {
                    synchronized (spitterInputs) {
                        sensorIO.updateInputs(sensorInputs);
                        spitterIO.updateInputs(spitterInputs);
                    }
                }
            }, sensorIO, spitterIO);
    }

    @Override
    public void periodic() {
        synchronized (spitterInputs) {
            synchronized (sensorInputs) {
                Logger.processInputs("Subsystems/Coralgun/Spitter", spitterInputs);
                Logger.processInputs("Subsystems/Coralgun/Sensor", sensorInputs);

                systemState = handleStateTransition();
                hasCoral = sensorInputs.sensorTripped;
                Logger.recordOutput("Subsystems/Coralgun/WantedState", wantedState);
                Logger.recordOutput("Subsystems/Coralgun/SystemState", systemState);

                applyStates();
            }
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
            case SPIT_L1 -> {
                return SystemState.SPITTING_L1;
            }
            case EJECT -> {
                return SystemState.EJECTING;
            }
            case PASSIVE_INTAKE -> {
                return SystemState.PASSIVE_INTAKING;
            }
            default -> {
                return SystemState.IDLING;
            }
        }  
    }

    public void applyStates() {
        switch (systemState) {
            case IDLING -> {
                spitterIO.stop();
                break;
            }
            case SPITTING -> {
                spitterIO.run(1);
                break;
            }
            case EJECTING -> {
                spitterIO.run(-1);
                break;
            } 
            case SPITTING_L1 -> {
                spitterIO.run(0.5);
                break;
            }
            case PASSIVE_INTAKING -> {
                spitterIO.run(-0.2);
                break;
            } 
             
        }
    }
}
