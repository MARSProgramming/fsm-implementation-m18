package frc.robot.subsystems.coral;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.util.SubsystemDataProcessor;

public interface SpitterIO extends SubsystemDataProcessor.IODataRefresher {
    @AutoLog
    class SpitterIOInputs {
        public double motorOutputPercent;
        public double motorStatorCurrent;
        public double motorTemperature;

    }

    default void updateInputs(SpitterIOInputs inputs) {}

    default void run(double speed) {}
    
    default void stop() {}
 
    @Override
    default void refreshData() {}
}
