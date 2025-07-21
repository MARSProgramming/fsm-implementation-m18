package frc.robot.subsystems.coral;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.SubsystemDataProcessor;

public class CoralSubsystem extends SubsystemBase {
    private final SpitterIO spitterIO;
    private final SensorIO sensorIO;

    private final SensorIOInputsAutoLogged sensorInputs = new SensorIOInputsAutoLogged();
    private final SpitterIOInputsAutoLogged spitterInputs = new SpitterIOInputsAutoLogged();

    private static final int SENSOR_DELAY_CYCLES = 10; // Number of cycles to confirm sensor state
    private int sensorTrueCounter = 0;
    private int sensorFalseCounter = 0;
    private boolean sensorState = false;

    private double spitterTimestamp = Double.NaN;
    private double spitterL1Timestamp = Double.NaN;
    private double ejectTimestamp = Double.NaN;

    private static final double SPIT_DURATION = 0.5;
    private static final double SPIT_L1_DURAITON = 0.5;
    private static final double EJECT_DURATION = 0.7;  // eject for longer as its going upwards the ramp. ejecting in general is probably useless, but good practice!

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

    public CoralSubsystem(SpitterIO spitIO, SensorIO sensIO) {
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

                updateSensorState();

                systemState = handleStateTransition();
                Logger.recordOutput("Subsystems/Coralgun/WantedState", wantedState);
                Logger.recordOutput("Subsystems/Coralgun/SystemState", systemState);

                applyStates();
            }
        }
    }

    private void updateSensorState() {
        if (sensorInputs.sensorTripped) {
            sensorTrueCounter++;
            sensorFalseCounter = 0;
        } else {
            sensorFalseCounter++;
            sensorTrueCounter = 0;
        }

        if (sensorTrueCounter >= SENSOR_DELAY_CYCLES) {
            sensorState = true;
        } else if (sensorFalseCounter >= SENSOR_DELAY_CYCLES) {
            sensorState = false;
        }

        Logger.recordOutput("Subsystems/Coralgun/SensorState", sensorState);
    }

    private SystemState handleStateTransition() {
        switch (wantedState) {
            case IDLE -> {
                return SystemState.IDLING;
            }
            case SPIT -> {
                if (!DriverStation.isDisabled()) {
                    if (Double.isNaN(spitterTimestamp)) {
                        spitterTimestamp = Timer.getFPGATimestamp();
                        return SystemState.SPITTING;
                    } else if ((Timer.getFPGATimestamp() - spitterTimestamp) > SPIT_DURATION) {
                        spitterTimestamp = Double.NaN;
                        setWantedState(WantedState.IDLE);
                        return SystemState.IDLING;
                    } else {
                        return SystemState.SPITTING; // <-- stay in SPITTING until duration passes
                    }
                } else {
                    spitterTimestamp = Double.NaN; // optional: reset if disabled
                    return SystemState.IDLING;
                }
            }
            
            case SPIT_L1 -> {
                if (!DriverStation.isDisabled()) {
                    if (Double.isNaN(spitterL1Timestamp)) {
                        spitterL1Timestamp = Timer.getFPGATimestamp();
                        return SystemState.SPITTING;
                    } else if ((Timer.getFPGATimestamp() - spitterL1Timestamp) > SPIT_L1_DURAITON) {
                        spitterL1Timestamp = Double.NaN;
                        setWantedState(WantedState.IDLE);
                        return SystemState.IDLING;
                    } else {
                        return SystemState.SPITTING; // <-- stay in SPITTING until duration passes
                    }
                } else {
                    spitterL1Timestamp = Double.NaN; // optional: reset if disabled
                    return SystemState.IDLING;
                }
            }
            case EJECT -> {
                if (!DriverStation.isDisabled()) {
                    if (Double.isNaN(ejectTimestamp)) {
                        ejectTimestamp = Timer.getFPGATimestamp();
                        return SystemState.SPITTING;
                    } else if ((Timer.getFPGATimestamp() - ejectTimestamp) > EJECT_DURATION) {
                        ejectTimestamp = Double.NaN;
                        setWantedState(WantedState.IDLE);
                        return SystemState.IDLING;
                    } else {
                        return SystemState.SPITTING; // <-- stay in SPITTING until duration passes
                    }
                } else {
                    ejectTimestamp = Double.NaN; // optional: reset if disabled
                    return SystemState.IDLING;
                }
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

    public void setWantedState(WantedState wantedState) {
        this.wantedState = wantedState;
    }

    public boolean hasCoral() {
        return sensorState;
    }
}
