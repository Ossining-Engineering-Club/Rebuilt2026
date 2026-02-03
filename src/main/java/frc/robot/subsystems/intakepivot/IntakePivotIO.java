package frc.robot.subsystems.intakepivot;

import org.littletonrobotics.junction.AutoLog;

public interface IntakePivotIO {
    @AutoLog
    public static class IntakePivotIOInputs {
        public double appliedVolts = 0.0;
        public double angleRadians = 0.0;
        public double statorCurrent = 0.0;
        public double temperatureCelsius = 0.0;
    }

    /** Updates the set of loggable inputs */
    public default void updateInputs(IntakePivotIOInputs inputs) {}

    /** Sets voltage of motor */
    public default void setVoltage(double voltage) {}

    public default void resetSimState() {}
}