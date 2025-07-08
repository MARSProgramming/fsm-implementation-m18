package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.util.SubsystemDataProcessor;

public interface LimiterIO extends SubsystemDataProcessor.IODataRefresher {
    @AutoLog
    class LimiterIOInputs {
        public boolean limit;
        public double servoAngle;
        public double servoPosition;
    }

    default void targetServoAngle(double angle) {}

    default void targetServoPosition(double position) {}

    default void updateInputs(LimiterIOInputs inputs) {}

    @Override
    default void refreshData() {}
}
