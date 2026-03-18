package frc.robot.subsystems.feeder;

import static frc.robot.subsystems.feeder.FeederConstants.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import org.littletonrobotics.junction.Logger;

public class FeederIOTalonFX implements FeederIO {
  private final TalonFX feederMotor;

  public FeederIOTalonFX() {
    feederMotor = new TalonFX(FeederConstants.feederCANID);

    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.CurrentLimits.SupplyCurrentLimit = feederMotorSupplyCurrentLimit;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.Inverted =
        isInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

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
    double appliedVolts = MathUtil.clamp(voltage, -12.0, 12.0);
    Logger.recordOutput("SetVoltages/Feeder", appliedVolts);
    feederMotor.setVoltage(appliedVolts);
  }
}
