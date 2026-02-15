package frc.robot.subsystems.shooterflywheels;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterFlywheelsIO {
    @AutoLog
    public static class ShooterFlywheelsIOInputs {
        public double appliedVolts = 0.0;
        public double statorCurrent = 0.0;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ShooterFlywheelsIOInputs inputs) {}

    /** Sets voltage of motor */
    public default void setRollersMotorVoltage(double voltage) {}
}
