package frc.robot.subsystems.algae;

import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.constants.RobotMap;

public class GrabberIOSparkMax implements GrabberIO {
  private final SparkBase grabber;
  private final SparkBaseConfig config;

  private final int currentLimit = 40;

  public GrabberIOSparkMax() {
    grabber = new SparkMax(RobotMap.SPARK.ALGAE, MotorType.kBrushless);
    config = new SparkMaxConfig();
    config.smartCurrentLimit(currentLimit);
    config.signals.busVoltagePeriodMs(20).outputCurrentPeriodMs(20).appliedOutputPeriodMs(20);

    tryUntilOk(
        grabber,
        5,
        () ->
            grabber.configure(
                config,
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(GrabberIOInputs inputs) {
    inputs.motorOutputPercent = grabber.getAppliedOutput();
    inputs.motorBusVoltage = grabber.getBusVoltage();
    inputs.motorTemperature = grabber.getMotorTemperature();
  }

  @Override
  public void run(double percent) {
    grabber.set(percent);
  }

  @Override
  public void stop() {
    grabber.set(0);
  }
}