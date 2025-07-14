package frc.robot.subsystems.coralgun;

import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.constants.Constants;
import frc.robot.constants.RobotMap;

public class SensorIORoboRIO implements SensorIO {
    private final AnalogInput ir;

    public SensorIORoboRIO() {
        ir = new AnalogInput(RobotMap.RIO.IR);
        ir.setAverageBits(4);
        ir.setOversampleBits(4);
    }

    private int integ = 0;

    @Override
    public void updateInputs(SensorIOInputs inputs) {
        inputs.sensorAverageVoltage = ir.getAverageVoltage();
        if (ir.getAverageValue() > Constants.IntakeConstants.CoralGunConstants.IR_SENSOR_THRESHOLD) {
            integ++;
        } else {
            integ = 0;
        }
        if (integ >= 5) {
            inputs.sensorTripped = true;
        } else {
            inputs.sensorTripped = false;
        }
    }

    @Override
    public void refreshData() {}
}

