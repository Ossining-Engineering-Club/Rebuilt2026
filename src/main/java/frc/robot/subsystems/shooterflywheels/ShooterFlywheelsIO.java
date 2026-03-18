package frc.robot.subsystems.shooterflywheels;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterFlywheelsIO {
  @AutoLog
  public static class ShooterFlywheelsIOInputs {
    public double RPM = 0.0;

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
  public default void updateInputs(ShooterFlywheelsIOInputs inputs) {}

  /** Sets RPM of flywheel. */
  public default void setRPM(double RPM) {}

  /** Sets voltage of flywheel motors. */
  public default void setVoltage(double voltage) {}
}
