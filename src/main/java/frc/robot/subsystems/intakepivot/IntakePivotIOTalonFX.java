package frc.robot.subsystems.intakepivot;

import static frc.robot.subsystems.intakepivot.IntakePivotConstants.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import org.littletonrobotics.junction.Logger;

public class IntakePivotIOTalonFX implements IntakePivotIO {
  private final TalonFX pivotMotor;

  public IntakePivotIOTalonFX() {
    pivotMotor = new TalonFX(canid);

    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.CurrentLimits.SupplyCurrentLimit = supplyCurrentLimit;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.Feedback.SensorToMechanismRatio = sensorMechanismRatio;
    config.MotorOutput.Inverted =
        isInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

    tryUntilOk(5, () -> pivotMotor.getConfigurator().apply(config, 0.25));
    tryUntilOk(5, () -> pivotMotor.setPosition(startAngle, 0.25));
  }

  @Override
  public void updateInputs(IntakePivotIOInputs inputs) {
    inputs.appliedVolts = pivotMotor.getMotorVoltage().getValueAsDouble();
    inputs.angleRadians = pivotMotor.getPosition().getValueAsDouble();
    inputs.statorCurrent = pivotMotor.getStatorCurrent().getValueAsDouble();
    inputs.temperatureCelsius = pivotMotor.getDeviceTemp().getValueAsDouble();
  }

  @Override
  public void setVoltage(double voltage) {
    double appliedVolts = MathUtil.clamp(voltage, -12.0, 12.0);
    Logger.recordOutput("SetVoltages/IntakePivot", appliedVolts);
    pivotMotor.setVoltage(appliedVolts);
  }
}
