package frc.robot.subsystems.intakepivot;

import static frc.robot.subsystems.intakepivot.IntakePivotConstants.*;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import org.littletonrobotics.junction.Logger;

public class IntakePivotIOReal implements IntakePivotIO {
  private final SparkMax sparkMax;
  private final RelativeEncoder encoder;

  public IntakePivotIOReal() {
    sparkMax = new SparkMax(canid, MotorType.kBrushless);
    encoder = sparkMax.getEncoder();

    var config = new SparkMaxConfig();
    config.inverted(isInverted).idleMode(IdleMode.kBrake);
    config
        .encoder
        .positionConversionFactor(1.0 / motorReduction * encoderPositionFactor)
        .velocityConversionFactor(1.0 / motorReduction * encoderVelocityFactor);
    config.smartCurrentLimit(stallCurrentLimit, freeCurrentLimit);
    tryUntilOk(
        sparkMax,
        5,
        () ->
            sparkMax.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    encoder.setPosition(startAngle);
  }

  @Override
  public void updateInputs(IntakePivotIOInputs inputs) {
    inputs.appliedVolts = sparkMax.getAppliedOutput() * sparkMax.getBusVoltage();
    inputs.angleRadians = encoder.getPosition();
    inputs.statorCurrent = sparkMax.getOutputCurrent();
    inputs.temperatureCelsius = sparkMax.getMotorTemperature();
  }

  @Override
  public void setVoltage(double voltage) {
    double appliedVolts = MathUtil.clamp(voltage, -12.0, 12.0);
    Logger.recordOutput("intake pivot set voltage", appliedVolts);
    sparkMax.setVoltage(appliedVolts);
  }
}
