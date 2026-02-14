package frc.robot.subsystems.feeder;

import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class FeederIOSparkMax implements FeederIO {
  private final SparkMax feederMotor;

  public FeederIOSparkMax() {
    feederMotor = new SparkMax(FeederConstants.feederCANID, MotorType.kBrushless);

    var feederMotorConfig = new SparkMaxConfig();
    feederMotorConfig.inverted(FeederConstants.isInverted).idleMode(IdleMode.kBrake);
    feederMotorConfig.smartCurrentLimit(
        FeederConstants.feederMotorStallCurrentLimit, FeederConstants.feederMotorFreeCurrentLimit);

    tryUntilOk(
        feederMotor,
        5,
        () ->
            feederMotor.configure(
                feederMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(FeederIOInputs inputs) {
    inputs.appliedVolts = feederMotor.getAppliedOutput() * feederMotor.getBusVoltage();
    inputs.statorCurrent = feederMotor.getOutputCurrent();
  }

  @Override
  public void setFeederMotorVoltage(double voltage) {
    feederMotor.setVoltage(voltage);
  }
}
