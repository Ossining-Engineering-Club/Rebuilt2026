package frc.robot.subsystems.shooterflywheels;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.shooterflywheels.ShooterFlywheelsConstants.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ShooterFlywheelsIOReal implements ShooterFlywheelsIO {
  private final TalonFX leftMotor; // leader
  private final TalonFX rightMotor; // follower

  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0); // RPM control
  private final VoltageOut voltageRequest = new VoltageOut(0); // directly set voltage

  public ShooterFlywheelsIOReal() {
    leftMotor = new TalonFX(leftMotorCanId);
    rightMotor = new TalonFX(rightMotorCanId);

    var leftConfig = new TalonFXConfiguration();
    leftConfig.MotorOutput.Inverted =
        isInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    leftConfig.Feedback.SensorToMechanismRatio = motorReduction;
    leftConfig.CurrentLimits.SupplyCurrentLimit = supplyCurrentLimit;
    leftConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    var slot0Configs = new Slot0Configs();
    slot0Configs.kP = kP;
    slot0Configs.kI = kI;
    slot0Configs.kD = kD;
    slot0Configs.kS = kS;
    slot0Configs.kV = kV;
    leftConfig.Slot0 = slot0Configs;

    tryUntilOk(5, () -> leftMotor.getConfigurator().apply(leftConfig, 0.25));

    var rightConfig = new TalonFXConfiguration();
    rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rightConfig.Feedback.SensorToMechanismRatio = motorReduction;
    rightConfig.CurrentLimits.SupplyCurrentLimit = supplyCurrentLimit;
    rightConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    tryUntilOk(5, () -> rightMotor.getConfigurator().apply(rightConfig, 0.25));

    // set right motor to follow left motor
    rightMotor.setControl(new Follower(leftMotor.getDeviceID(), MotorAlignmentValue.Opposed));
  }

  @Override
  public void updateInputs(ShooterFlywheelsIOInputs inputs) {
    inputs.RPM = leftMotor.getVelocity().getValueAsDouble();

    inputs.leftAppliedVolts = leftMotor.getMotorVoltage().getValueAsDouble();
    inputs.leftStatorCurrent = leftMotor.getStatorCurrent().getValueAsDouble();
    inputs.leftSupplyCurrent = leftMotor.getSupplyCurrent().getValueAsDouble();
    inputs.leftTemperatureCelsius = leftMotor.getDeviceTemp().getValueAsDouble();

    inputs.rightAppliedVolts = rightMotor.getMotorVoltage().getValueAsDouble();
    inputs.rightStatorCurrent = rightMotor.getStatorCurrent().getValueAsDouble();
    inputs.rightSupplyCurrent = rightMotor.getSupplyCurrent().getValueAsDouble();
    inputs.rightTemperatureCelsius = rightMotor.getDeviceTemp().getValueAsDouble();
  }

  @Override
  public void setRPM(double RPM) {
    double desiredRotPerSec = RPM / 60.0;
    leftMotor.setControl(velocityRequest.withVelocity(desiredRotPerSec));
  }

  @Override
  public void setVoltage(double voltage) {
    leftMotor.setControl(voltageRequest.withOutput(Volts.of(voltage)));
  }
}
