package frc.robot.subsystems.intakerollers;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeRollersIO {
    @AutoLog
    public static class IntakeRollersIOInputs {
        public double appliedVolts = 0.0;
        public double statorCurrent = 0.0;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(IntakeRollersIOInputs inputs) {}

    /** Sets voltage of motor */
    public default void setRollersMotorVoltage(double voltage) {}
}
