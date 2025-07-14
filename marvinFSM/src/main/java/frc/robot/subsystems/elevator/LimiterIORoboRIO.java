package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.constants.RobotMap;

public class LimiterIORoboRIO implements LimiterIO {
    private final DigitalInput limitSwitch;
    private final Servo climbServo;

    public LimiterIORoboRIO() {
        limitSwitch = new DigitalInput(RobotMap.DIO.LIMIT);
        climbServo = new Servo(RobotMap.PWM.SERVO);
    }

    @Override
    public void targetServoAngle(double angle) {
        climbServo.setAngle(angle);
    }

    @Override
    public void targetServoPosition(double position) {
        climbServo.set(position);
    }

    @Override
    public void updateInputs(LimiterIOInputs inputs) {
        inputs.limit = !limitSwitch.get();
        inputs.servoAngle = climbServo.getAngle();
        inputs.servoPosition = climbServo.getPosition();
    }
}
