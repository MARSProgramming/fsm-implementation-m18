package frc.robot.subsystems.coral;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.constants.RobotMap;

public class SpitterIOTalonSRX implements SpitterIO {
    private final TalonSRX motor;
    
    public SpitterIOTalonSRX() {
        motor = new TalonSRX(RobotMap.CAN.CORAL);

        motor.configFactoryDefault();
        motor.setNeutralMode(NeutralMode.Brake); // Could this prevent coral from falling out of the mechanism during auto...
    }

    @Override
    public void updateInputs(SpitterIOInputs inputs) {
        inputs.motorOutputPercent = motor.getMotorOutputPercent();
        inputs.motorStatorCurrent = motor.getStatorCurrent();
        inputs.motorTemperature = motor.getTemperature();
    }

    @Override
    public void run(double speed) {
        motor.set(ControlMode.PercentOutput, speed);
    }

    @Override
    public void stop() {
        motor.set(ControlMode.PercentOutput, 0.0);
    }
}
