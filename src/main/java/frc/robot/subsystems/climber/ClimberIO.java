package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  public static class ClimberIOInputs {
    public double climberMotorAppliedVolts = 0;
    public double climberPosition = ClimberConstants.startPosition;
    public double climberMotorStatorCurrent = 0.0;
    public double climberMotorTemperatureCelsius = 0.0;
    public double climberMotorAngleRadians = 0.0;
  }

  public default void updateInputs(ClimberIOInputs inputs) {}

  public default void setClimberMotorVoltage(double voltage) {}
}
