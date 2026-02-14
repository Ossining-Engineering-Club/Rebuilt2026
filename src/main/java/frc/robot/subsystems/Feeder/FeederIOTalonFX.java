package frc.robot.subsystems.feeder;

import static frc.robot.subsystems.feeder.FeederConstants.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class FeederIOTalonFX implements FeederIO {
  private final TalonFX feederMotor;

  public FeederIOTalonFX() {
    feederMotor = new TalonFX(FeederConstants.feederCANID);

    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.CurrentLimits.SupplyCurrentLimit = feederMotorSupplyCurrentLimit;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    tryUntilOk(5, () -> feederMotor.getConfigurator().apply(config, 0.25));
  }

  @Override
  public void updateInputs(FeederIOInputs inputs) {
    inputs.appliedVolts = feederMotor.getMotorVoltage().getValueAsDouble();
    inputs.statorCurrent = feederMotor.getStatorCurrent().getValueAsDouble();
    inputs.supplyCurrent = feederMotor.getSupplyCurrent().getValueAsDouble();
    inputs.temperatureCelsius = feederMotor.getDeviceTemp().getValueAsDouble();
  }

  @Override
  public void setFeederMotorVoltage(double voltage) {
    feederMotor.setVoltage(voltage);
  }
}
