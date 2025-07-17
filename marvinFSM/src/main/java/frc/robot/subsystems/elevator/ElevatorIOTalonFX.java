package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.RobotMap;

public class ElevatorIOTalonFX implements ElevatorIO {
    private final TalonFX master;
    private final TalonFX follower;

    private final MotionMagicVoltage positionRequest;
    private final VoltageOut voltageRequest;

    private final StatusSignal<Voltage> elevAppliedVolts;
    private final StatusSignal<Angle> elevPosition;
    private final StatusSignal<Current> elevSupplyCurrent;
    private final StatusSignal<Current> elevStatorCurrent;
    private final StatusSignal<AngularVelocity> elevAngularVelocityRadPerSec;
    private final StatusSignal<AngularAcceleration> elevAngularAccelerationRadPerSecSquared;

    private final StatusSignal<Temperature> elevOneMotorTemp;
    private final StatusSignal<Temperature> elevTwoMotorTemp;

    public ElevatorIOTalonFX() {
        master = new TalonFX(RobotMap.CAN.ELEVATOR_MASTER, "CAN-2");
        follower = new TalonFX(RobotMap.CAN.ELEVATOR_FOLLOWER, "CAN-2");
        var masterConfig = new TalonFXConfiguration();
        
        masterConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        masterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        masterConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 8; // update constants file
        masterConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
        masterConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false; // TESTING ONLY
        masterConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false; // TESTING ONLY
    // masterConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;
        masterConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        masterConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        masterConfig.CurrentLimits.SupplyCurrentLimit = 54;
        masterConfig.CurrentLimits.SupplyCurrentLowerLimit = 30;
        masterConfig.CurrentLimits.SupplyCurrentLowerTime = 1;

        masterConfig.CurrentLimits.StatorCurrentLimit = 120;
        masterConfig.Feedback.SensorToMechanismRatio = 12;
        masterConfig.Voltage.PeakForwardVoltage = 16;
        masterConfig.Voltage.PeakReverseVoltage = -16;

        masterConfig.Slot0.kP = 18;
        masterConfig.Slot0.kI = 2;
        masterConfig.Slot0.kD = 2;
        masterConfig.Slot0.kG = 0.3;
        masterConfig.Slot0.kS = 0.1;
        masterConfig.MotionMagic.MotionMagicCruiseVelocity = 45;
        masterConfig.MotionMagic.MotionMagicAcceleration = 110;

        master.getConfigurator().apply(masterConfig);
        follower.getConfigurator().apply(masterConfig);

        follower.setControl(new Follower(master.getDeviceID(), true));

        elevAppliedVolts = master.getSupplyVoltage();
        elevPosition = master.getPosition();
        elevSupplyCurrent = master.getSupplyCurrent();
        elevStatorCurrent = master.getStatorCurrent();
        elevAngularVelocityRadPerSec = master.getVelocity();
        elevAngularAccelerationRadPerSecSquared = master.getAcceleration();

        elevOneMotorTemp = master.getDeviceTemp();
        elevTwoMotorTemp = follower.getDeviceTemp();

        positionRequest = new MotionMagicVoltage(0);
        voltageRequest = new VoltageOut(0);
        
        positionRequest.EnableFOC = true;
        voltageRequest.EnableFOC = true;
    }

    @Override
    public void setTargetPosition(double position) {
        master.setControl(positionRequest.withPosition(position));
    }

    @Override
    public void climb() {
        master.setControl(voltageRequest.withOutput(-5)); // make a constant later
    }

    @Override
    public void applyVoltage(double out) {
        master.setControl(new VoltageOut(out));
    }


    @Override
    public void stop() {
        master.set(0);
    }

    @Override
    public void resetPosition() {
        master.setPosition(0);
    }

    @Override
    public void setNeutralMode(NeutralModeValue neutralMode) {
        master.setNeutralMode(neutralMode);
        follower.setNeutralMode(neutralMode);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.elevAppliedVolts = master.getSupplyVoltage().getValueAsDouble();
        inputs.elevPosition = master.getPosition().getValueAsDouble();
        inputs.elevSupplyCurrentAmps = master.getSupplyCurrent().getValueAsDouble();
        inputs.elevStatorCurrentAmps = master.getStatorCurrent().getValueAsDouble();
        inputs.elevAngularVelocityRadPerSec = master.getVelocity().getValueAsDouble();
        inputs.elevAngularAccelerationRadPerSecSquared = master.getAcceleration().getValueAsDouble();
        inputs.elevOneMotorTemp = master.getDeviceTemp().getValueAsDouble();
        inputs.elevTwoMotorTemp = follower.getDeviceTemp().getValueAsDouble();
    }

    @Override
    public void refreshData() {
        BaseStatusSignal.refreshAll(
            elevAppliedVolts,
            elevSupplyCurrent,
            elevPosition,
            elevStatorCurrent,
            elevAngularVelocityRadPerSec,
            elevAngularAccelerationRadPerSecSquared,
            elevOneMotorTemp,
            elevTwoMotorTemp
        );

    }

}
