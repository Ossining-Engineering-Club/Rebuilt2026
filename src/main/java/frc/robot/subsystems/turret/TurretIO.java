package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
  @AutoLog
  public static class TurretIOInputs {
    public double appliedVolts = 0.0;
    public double angleRadians = 0.0;
    public double angularVelocityRadPerSec = 0.0;
    public double absEncoder18tReading = 0.0;
    public double absEncoder19tReading = 0.0;
    public double absAngleRadians = 0.0;
    public double statorCurrent = 0.0;
    public double temperatureCelsius = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(TurretIOInputs inputs) {}

  /** Sets the voltage of the turret motor. */
  public default void setVoltage(double voltage) {}

  public default void resetSimState() {}
}
