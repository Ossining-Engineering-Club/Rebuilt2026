package frc.robot.subsystems.intakerollers;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeRollersIO {
  @AutoLog
  public static class IntakeRollersIOInputs {
    public double leftAppliedVolts = 0.0;
    public double leftStatorCurrent = 0.0;
    public double leftSupplyCurrent = 0.0;
    public double leftTemperatureCelsius = 0.0;

    public double rightAppliedVolts = 0.0;
    public double rightStatorCurrent = 0.0;
    public double rightSupplyCurrent = 0.0;
    public double rightTemperatureCelsius = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(IntakeRollersIOInputs inputs) {}

  /** Sets voltage of motor */
  public default void setRollersMotorVoltage(double voltage) {}
}
