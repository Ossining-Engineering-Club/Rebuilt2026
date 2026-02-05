package frc.robot.subsystems.climber;

import static frc.robot.subsystems.climber.ClimberConstants.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ClimberIOReal implements ClimberIO {
  private final TalonFX climberMotor;

  public ClimberIOReal() {
    climberMotor = new TalonFX(ClimberConstants.climberMotorCanId);

    var config = new TalonFXConfiguration();
    config.MotorOutput.Inverted =
        isInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.CurrentLimits.StatorCurrentLimit = currentLimit;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    tryUntilOk(5, () -> climberMotor.getConfigurator().apply(config, 0.25));
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.climberMotorAppliedVolts = climberMotor.getMotorVoltage().getValueAsDouble();
    inputs.climberPosition = climberMotor.getPosition().getValueAsDouble();
    inputs.climberMotorStatorCurrent = climberMotor.getStatorCurrent().getValueAsDouble();
    inputs.climberMotorTemperatureCelsius = climberMotor.getDeviceTemp().getValueAsDouble();
  }

  @Override
  public void setClimberMotorVoltage(double voltage) {
    climberMotor.setVoltage(voltage);
  }
}
