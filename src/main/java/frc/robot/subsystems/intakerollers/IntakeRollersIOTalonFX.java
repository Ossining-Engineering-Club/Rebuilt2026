package frc.robot.subsystems.intakerollers;

import static frc.robot.subsystems.intakerollers.IntakeRollersConstants.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IntakeRollersIOTalonFX implements IntakeRollersIO {
  private final TalonFX rollersMotor;

  public IntakeRollersIOTalonFX() {
    rollersMotor = new TalonFX(intakeRollersCANID);

    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.CurrentLimits.SupplyCurrentLimit = rollersMotorSupplyCurrentLimit;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    tryUntilOk(5, () -> rollersMotor.getConfigurator().apply(config, 0.25));
  }

  @Override
  public void updateInputs(IntakeRollersIOInputs inputs) {
    inputs.appliedVolts = rollersMotor.getMotorVoltage().getValueAsDouble();
    inputs.statorCurrent = rollersMotor.getStatorCurrent().getValueAsDouble();
    inputs.supplyCurrent = rollersMotor.getSupplyCurrent().getValueAsDouble();
    inputs.temperatureCelsius = rollersMotor.getDeviceTemp().getValueAsDouble();
  }

  @Override
  public void setRollersMotorVoltage(double voltage) {
    rollersMotor.setVoltage(voltage);
  }
}
