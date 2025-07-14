package frc.robot.subsystems.coralgun;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.util.SubsystemDataProcessor;

public interface SensorIO extends SubsystemDataProcessor.IODataRefresher {
    @AutoLog
    class SensorIOInputs {
        public double sensorAverageVoltage;
        public boolean sensorTripped;
    }

    default void updateInputs(SensorIOInputs inputs) {}
}
