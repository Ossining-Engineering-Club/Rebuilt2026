package frc.robot.subsystems.intakepivot;

import static frc.robot.subsystems.intakepivot.IntakePivotConstants.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import org.littletonrobotics.junction.Logger;

public class IntakePivotIOTalonFX implements IntakePivotIO {
  private final TalonFX rollersMotor;

  public IntakePivotIOTalonFX() {
    rollersMotor = new TalonFX(canid);

    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.CurrentLimits.SupplyCurrentLimit = supplyCurrentLimit;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    tryUntilOk(5, () -> rollersMotor.getConfigurator().apply(config, 0.25));
  }

  @Override
  public void updateInputs(IntakePivotIOInputs inputs) {
    inputs.appliedVolts = rollersMotor.getMotorVoltage().getValueAsDouble();
    inputs.angleRadians = rollersMotor.getPosition().getValueAsDouble();
    inputs.statorCurrent = rollersMotor.getStatorCurrent().getValueAsDouble();
    inputs.temperatureCelsius = rollersMotor.getDeviceTemp().getValueAsDouble();
  }

  @Override
  public void setVoltage(double voltage) {
    double appliedVolts = MathUtil.clamp(voltage, -12.0, 12.0);
    Logger.recordOutput("SetVoltages/IntakeRollers", appliedVolts);
    rollersMotor.setVoltage(appliedVolts);
  }
}
