package frc.robot.subsystems.shooterhood;

import static frc.robot.subsystems.shooterhood.ShooterHoodConstants.*;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.MathUtil;
import org.littletonrobotics.junction.Logger;

public class ShooterHoodIOReal implements ShooterHoodIO {
  private final SparkFlex hoodMotor;
  private final RelativeEncoder encoder;

  public ShooterHoodIOReal() {
    hoodMotor = new SparkFlex(shooterHoodCANID, MotorType.kBrushless);
    encoder = hoodMotor.getEncoder();

    var config = new SparkFlexConfig();
    config.inverted(isInverted).idleMode(IdleMode.kBrake);
    config
        .encoder
        .positionConversionFactor(1.0 / motorReduction * encoderPositionFactor)
        .velocityConversionFactor(1.0 / motorReduction * encoderVelocityFactor);
    config.smartCurrentLimit(stallCurrentLimit, freeCurrentLimit);
    tryUntilOk(
        hoodMotor,
        5,
        () ->
            hoodMotor.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    encoder.setPosition(startAngle);
  }

  @Override
  public void updateInputs(ShooterHoodIOInputs inputs) {
    inputs.appliedVolts = hoodMotor.getAppliedOutput() * hoodMotor.getBusVoltage();
    inputs.angleRadians = encoder.getPosition();
    inputs.statorCurrent = hoodMotor.getOutputCurrent();
    inputs.temperatureCelsius = hoodMotor.getMotorTemperature();
  }

  @Override
  public void setVoltage(double voltage) {
    double appliedVolts = MathUtil.clamp(voltage, -12.0, 12.0);
    Logger.recordOutput("SetVoltages/ShooterHood", appliedVolts);
    hoodMotor.setVoltage(appliedVolts);
  }
}
