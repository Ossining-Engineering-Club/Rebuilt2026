package frc.robot.subsystems.intakerollers;

import static frc.robot.subsystems.intakerollers.IntakeRollersConstants.*;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import org.littletonrobotics.junction.Logger;

public class IntakeRollersIOSparkMax implements IntakeRollersIO {
  private final SparkMax rollersMotor;

  public IntakeRollersIOSparkMax() {
    rollersMotor = new SparkMax(intakeRollersCANID, MotorType.kBrushless);

    var intakeRollersMotorConfig = new SparkMaxConfig();
    intakeRollersMotorConfig.inverted(isInverted).idleMode(IdleMode.kBrake);
    intakeRollersMotorConfig.smartCurrentLimit(
        rollersMotorStallCurrentLimit, rollersMotorFreeCurrentLimit);
    tryUntilOk(
        rollersMotor,
        5,
        () ->
            rollersMotor.configure(
                intakeRollersMotorConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(IntakeRollersIOInputs inputs) {
    inputs.appliedVolts = rollersMotor.getAppliedOutput() * rollersMotor.getBusVoltage();
    inputs.statorCurrent = rollersMotor.getOutputCurrent();
  }

  @Override
  public void setRollersMotorVoltage(double voltage) {
    double appliedVolts = MathUtil.clamp(voltage, -12.0, 12.0);
    Logger.recordOutput("SetVoltages/IntakeRollers", appliedVolts);
    rollersMotor.setVoltage(appliedVolts);
  }
}
