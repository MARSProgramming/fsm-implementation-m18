package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.SubsystemDataProcessor;

public class ElevatorSubsystem extends SubsystemBase {
    private final ElevatorIO elevatorIO;
    private final LimiterIO limitIO;

    private final ElevatorIOInputsAutoLogged elevatorInputs = new ElevatorIOInputsAutoLogged();
    private final LimiterIOInputsAutoLogged limiterInputs = new LimiterIOInputsAutoLogged();

    private boolean isElevatorHomed = false;
    private boolean homedSafely = false;
    private double elevatorHomeTimestamp = Double.NaN;

    private double elevatorSetpoint = 0.0;
    private double elevatorVoltageSetpoint = 0.0;

    public enum WantedState {
        HOME,
        IDLE,
        MOVE_TO_POSITION,
        HOLD_POSITION,
        OPEN_SERVO,
        CLIMB
    }

    private enum SystemState {
        HOMING,
        IDLING,
        MOVING_TO_POSITION,
        HOLDING_POSITION,
        OPENING_SERVO,
        CLIMBING
    }

    private WantedState wantedState = WantedState.IDLE;
    private WantedState previousWantedState = WantedState.IDLE;
    private SystemState systemState = SystemState.IDLING;

    public ElevatorSubsystem(ElevatorIO elevio, LimiterIO limio) {
        elevatorIO = elevio;
        limitIO = limio;

        SubsystemDataProcessor.createAndStartSubsystemDataProcessor(
        () -> {
            synchronized (elevatorInputs) {
                synchronized (limiterInputs) {
                    elevatorIO.updateInputs(elevatorInputs);
                    limitIO.updateInputs(limiterInputs);
                }
            }
        }, elevatorIO, limitIO);
    }

    // leave room for periodic here
    @Override
    public void periodic() {
        synchronized (elevatorInputs) {
            synchronized (limiterInputs) {
                Logger.processInputs("Subsystems/Elevator/Main", elevatorInputs);
                Logger.processInputs("Subystems/Elevator/DIOandRIO", elevatorInputs);

                systemState = handleStateTransition();
                Logger.recordOutput("Subsystems/Elevator/WantedState", wantedState);
                Logger.recordOutput("Subsystems/Elevator/SystemState", systemState);
                Logger.recordOutput("Subsystems/Elevator/HomedSafely", homedSafely);
                double wantedElevatorSetpoint;
                double wantedElevatorVoltage;

                wantedElevatorSetpoint = this.elevatorSetpoint;
                wantedElevatorVoltage = this.elevatorVoltageSetpoint;

                Logger.recordOutput("Subsystems/Elevator/DesiredSetpoint", wantedElevatorSetpoint);
                Logger.recordOutput("Subsystems/Elevator/DesiredVoltage", wantedElevatorVoltage);

                applyStates();
                previousWantedState = this.wantedState;
            }
        }
    }
    private SystemState handleStateTransition() {
        if (!isElevatorHomed && wantedState != WantedState.HOME) {
            return SystemState.IDLING;
        }

        switch (wantedState) {
            case HOME -> {
    
                if (previousWantedState != WantedState.HOME) {
                    isElevatorHomed = false;
                }

                if (!DriverStation.isDisabled()) {
                    if (Math.abs(elevatorInputs.elevAngularVelocityRadPerSec) <= 0.5) {
                        if (Double.isNaN(elevatorHomeTimestamp)) {
                            elevatorHomeTimestamp = Timer.getFPGATimestamp();
                            return SystemState.HOMING;
                        }

                        if ((Timer.getFPGATimestamp() - elevatorHomeTimestamp) >= 3) { // maximum zeroing time
                            if (limiterInputs.limit && elevatorInputs.elevPosition < 0.1) {
                                isElevatorHomed = true;
                                homedSafely = true;
                                elevatorIO.resetPosition();

                                elevatorHomeTimestamp = Double.NaN;
                                setWantedState(WantedState.IDLE);
                                return SystemState.IDLING;
                            } else {
                                isElevatorHomed = true;
                                homedSafely = false;
                                elevatorHomeTimestamp = Double.NaN;
                                elevatorIO.resetPosition();

                                setWantedState(WantedState.IDLE);
                                return SystemState.IDLING;
                            }
                        }

                        return SystemState.HOMING;
                    } 
                } else {
                    return SystemState.IDLING;
                }
                
            } 

            case MOVE_TO_POSITION -> {
                return SystemState.MOVING_TO_POSITION;
            }

            case HOLD_POSITION -> {
                return SystemState.HOLDING_POSITION;
            } 

            case CLIMB -> {
                return SystemState.CLIMBING;
            }

            case OPEN_SERVO -> {
                return SystemState.OPENING_SERVO;
            }

            default -> {
                return SystemState.IDLING;
            }



        }

        return SystemState.IDLING;
    }

    public void applyStates() {
        switch (systemState) {
            case HOMING -> {
                limitIO.targetServoPosition(0);
                if (DriverStation.isAutonomous() && !DriverStation.isDisabled()) {
                    elevatorIO.applyVoltage(-6); // replace with a constant later
                } else {
                    elevatorIO.applyVoltage(elevatorVoltageSetpoint); // replace with a constant later
                }
                break;
            }

            case IDLING -> {
               //  limitIO.targetServoPosition(0); don't target this, after we climb servo should stay at 0.5
                elevatorIO.applyVoltage(0);
                break;
            }

            case CLIMBING -> {
                limitIO.targetServoPosition(0.5);
                elevatorIO.climb();
                break;
            }

            case MOVING_TO_POSITION -> {
                limitIO.targetServoPosition(0);
                elevatorIO.setTargetPosition(elevatorSetpoint);
                break;
            }

            case HOLDING_POSITION -> {
                limitIO.targetServoPosition(0);
                elevatorIO.setTargetPosition(elevatorInputs.elevPosition);
                break;
            }

            case OPENING_SERVO -> {
                limitIO.targetServoPosition(0.5);
            }
        }
    }



    public boolean reachedSetpoint() {
        synchronized (elevatorInputs) {
            return MathUtil.isNear(
                    this.elevatorSetpoint,
                    elevatorInputs.elevPosition,
                    0.05); // rotational tolerance, make a constant later
        }
    }

    public double getCurrentElevatorPositionRotations() {
        synchronized (elevatorInputs) {
            return elevatorInputs.elevPosition;
        }
    }

    public void setNeutralMode(NeutralModeValue neutralMode) {
        synchronized (elevatorInputs) {
            elevatorIO.setNeutralMode(neutralMode);
        }
    }

    public void setDesiredElevatorSetpoint(double position) {
        this.elevatorSetpoint = position;
        setWantedState(WantedState.MOVE_TO_POSITION);
    }

    public void zeroElevator(double position) {
        setWantedState(ElevatorSubsystem.WantedState.HOME);
    }

        
    public void setWantedState(WantedState wantedState) {
        this.wantedState = wantedState;
    }

    public void setWantedState(WantedState wantedState, double setpoint) {
        this.wantedState = wantedState;
        this.elevatorSetpoint = setpoint;
    }
    
    public void setDesiredElevatorVoltageForZeroing(double voltage) {
        setWantedState(WantedState.HOME);
        this.elevatorVoltageSetpoint = voltage;
    }

    public boolean hasHomeCompleted() {
        return isElevatorHomed;
    }

    public boolean limitRead() {
        return limiterInputs.limit;
    }

}
