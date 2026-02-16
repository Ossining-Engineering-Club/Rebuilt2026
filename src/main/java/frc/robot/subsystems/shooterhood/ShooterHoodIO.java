package frc.robot.subsystems.shooterhood;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterHoodIO {
  @AutoLog
  public static class ShooterHoodIOInputs {
    public double appliedVolts = 0.0;
    public double angleRadians = 0.0;
    public double statorCurrent = 0.0;
    public double temperatureCelsius = 0.0;
  }

  // Updates the set of Loggable inputs
  public default void updateInputs(ShooterHoodIOInputs inputs) {}

  // Sets voltage of motor
  public default void setVoltage(double voltage) {}

  public default void resetSimState() {}
}
