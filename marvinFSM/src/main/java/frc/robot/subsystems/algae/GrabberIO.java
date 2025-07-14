package frc.robot.subsystems.algae;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.util.SubsystemDataProcessor;

public interface GrabberIO extends SubsystemDataProcessor.IODataRefresher {
    @AutoLog
    class GrabberIOInputs {
        public double motorOutputPercent;
        public double motorBusVoltage;
        public double motorTemperature;

    }

    default void updateInputs(GrabberIOInputs inputs) {}

    default void run(double speed) {}
    
    default void stop() {}
 
    @Override
    default void refreshData() {}
}
