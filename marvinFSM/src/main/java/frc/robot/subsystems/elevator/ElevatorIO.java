package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.util.SubsystemDataProcessor;

public interface ElevatorIO extends SubsystemDataProcessor.IODataRefresher {
    
    @AutoLog
    class ElevatorIOInputs {
        public double elevPosition; // you can create two instances if you really want, but we only need to track one.
        public double elevAppliedVolts;
        public double elevSupplyCurrentAmps;
        public double elevStatorCurrentAmps;
        public double elevAngularVelocityRadPerSec;
        public double elevAngularAccelerationRadPerSecSquared;

        public double elevOneMotorTemp;
        public double elevTwoMotorTemp;

        
    }

    default void updateInputs(ElevatorIOInputs inputs) {}

    default void setTargetPosition(double pos) {}

    default void stop() {}

    default void applyVoltage(double out) {}

    default void resetPosition() {}

    default void climb() {}

    default void setNeutralMode(NeutralModeValue neturalMode) {}

    @Override
    default void refreshData() {}
}
