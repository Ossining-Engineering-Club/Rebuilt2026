package frc.robot.subsystems.turret;

import static frc.robot.subsystems.turret.TurretConstants.*;
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

public class TurretIOReal implements TurretIO {
  private final SparkFlex turretMotor;
  private final RelativeEncoder encoder;
  // private final DutyCycleEncoder absEncoder18t;
  // private final DutyCycleEncoder absEncoder19t;
  // private boolean hasEncoderPositionBeenSet = false;

  public TurretIOReal() {
    turretMotor = new SparkFlex(canId, MotorType.kBrushless);
    encoder = turretMotor.getEncoder();
    // absEncoder18t = new DutyCycleEncoder(AE18tChannel);
    // absEncoder19t = new DutyCycleEncoder(AE19tChannel);

    var config = new SparkFlexConfig();
    config.inverted(isInverted).idleMode(IdleMode.kBrake);
    config
        .encoder
        .positionConversionFactor(1.0 / motorReduction * encoderPositionFactor)
        .velocityConversionFactor(1.0 / motorReduction * encoderVelocityFactor);
    config.smartCurrentLimit(stallCurrentLimit, freeCurrentLimit);
    tryUntilOk(
        turretMotor,
        5,
        () ->
            turretMotor.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    encoder.setPosition(startAngle);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    // Once the absolute encoders connect, update the turret motor encoder's position
    // if (!hasEncoderPositionBeenSet && absEncoder18t.isConnected() && absEncoder19t.isConnected())
    // {
    //   encoder.setPosition(
    //       ((absEncoder18t.get() - AE18tOffsetRotations)
    //               - (absEncoder19t.get() - AE19tOffsetRotations))
    //           * AEDifferenceMultiplier);
    //   hasEncoderPositionBeenSet = true;
    // }

    inputs.appliedVolts = turretMotor.getAppliedOutput() * turretMotor.getBusVoltage();
    inputs.angleRadians = encoder.getPosition();
    inputs.angularVelocityRadPerSec = encoder.getVelocity();
    // inputs.absEncoder18tReading = absEncoder18t.get();
    // inputs.absEncoder19tReading = absEncoder19t.get();
    // inputs.absAngleRadians =
    //     ((absEncoder18t.get() - AE18tOffsetRotations + 1) % 1.0
    //             - (absEncoder19t.get() - AE19tOffsetRotations + 1) % 1.0)
    //         * AEDifferenceMultiplier;
    inputs.statorCurrent = turretMotor.getOutputCurrent();
    inputs.temperatureCelsius = turretMotor.getMotorTemperature();
  }

  @Override
  public void setVoltage(double voltage) {
    double appliedVolts = MathUtil.clamp(voltage, -12.0, 12.0);
    Logger.recordOutput("SetVoltages/Turret", appliedVolts);
    turretMotor.setVoltage(appliedVolts);
  }
}
