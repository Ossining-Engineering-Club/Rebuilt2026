package frc.robot.subsystems.spindexer;

import static frc.robot.subsystems.spindexer.SpindexerConstants.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import org.littletonrobotics.junction.Logger;

public class SpindexerIOReal implements SpindexerIO {
  private final TalonFX spindexerMotor;

  public SpindexerIOReal() {
    spindexerMotor = new TalonFX(SpindexerConstants.spindexerCanID);

    var config = new TalonFXConfiguration();
    config.MotorOutput.Inverted =
        isInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.Feedback.SensorToMechanismRatio = spindexerMotorReduction;
    config.CurrentLimits.SupplyCurrentLimit = spindexerMotorSupplyCurrentLimit;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    tryUntilOk(5, () -> spindexerMotor.getConfigurator().apply(config, 0.25));
  }

  @Override
  public void updateInputs(SpindexerIOInputs inputs) {
    inputs.appliedVolts = spindexerMotor.getMotorVoltage().getValueAsDouble();
    inputs.statorCurrent = spindexerMotor.getStatorCurrent().getValueAsDouble();
    inputs.supplyCurrent = spindexerMotor.getSupplyCurrent().getValueAsDouble();
    inputs.temperatureCelsius = spindexerMotor.getDeviceTemp().getValueAsDouble();
    inputs.velocityRotPerSec = spindexerMotor.getVelocity().getValueAsDouble();
  }

  @Override
  public void setSpindexerMotorVoltage(double voltage) {
    double appliedVolts = MathUtil.clamp(voltage, -12.0, 12.0);
    Logger.recordOutput("SetVoltages/Spindexer", appliedVolts);
    spindexerMotor.setVoltage(appliedVolts);
  }
}
