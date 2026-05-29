package frc.robot.subsystems.intakerollers;

import static frc.robot.subsystems.intakerollers.IntakeRollersConstants.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import org.littletonrobotics.junction.Logger;

public class IntakeRollersIOTalonFX implements IntakeRollersIO {
  private final TalonFX leftRollersMotor;
  private final TalonFX rightRollersMotor;

  public IntakeRollersIOTalonFX() {
    leftRollersMotor = new TalonFX(intakeRollersLeftCANID);
    rightRollersMotor = new TalonFX(intakeRollersRightCANID);

    var leftConfig = new TalonFXConfiguration();
    leftConfig.MotorOutput.Inverted =
        isInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    leftConfig.CurrentLimits.SupplyCurrentLimit = rollersMotorSupplyCurrentLimit;
    leftConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    tryUntilOk(5, () -> leftRollersMotor.getConfigurator().apply(leftConfig, 0.25));

    var rightConfig = new TalonFXConfiguration();
    rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    rightConfig.CurrentLimits.SupplyCurrentLimit = rollersMotorSupplyCurrentLimit;
    rightConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    tryUntilOk(5, () -> rightRollersMotor.getConfigurator().apply(rightConfig, 0.25));

    // set right motor to follow left motor
    rightRollersMotor.setControl(
        new Follower(leftRollersMotor.getDeviceID(), MotorAlignmentValue.Opposed));
  }

  @Override
  public void updateInputs(IntakeRollersIOInputs inputs) {
    inputs.leftAppliedVolts = leftRollersMotor.getMotorVoltage().getValueAsDouble();
    inputs.leftStatorCurrent = leftRollersMotor.getStatorCurrent().getValueAsDouble();
    inputs.leftSupplyCurrent = leftRollersMotor.getSupplyCurrent().getValueAsDouble();
    inputs.leftTemperatureCelsius = leftRollersMotor.getDeviceTemp().getValueAsDouble();

    inputs.rightAppliedVolts = rightRollersMotor.getMotorVoltage().getValueAsDouble();
    inputs.rightStatorCurrent = rightRollersMotor.getStatorCurrent().getValueAsDouble();
    inputs.rightSupplyCurrent = rightRollersMotor.getSupplyCurrent().getValueAsDouble();
    inputs.rightTemperatureCelsius = rightRollersMotor.getDeviceTemp().getValueAsDouble();
  }

  @Override
  public void setRollersMotorVoltage(double voltage) {
    double appliedVolts = MathUtil.clamp(voltage, -12.0, 12.0);
    Logger.recordOutput("SetVoltages/IntakeRollers", appliedVolts);
    leftRollersMotor.setVoltage(appliedVolts);
  }
}
