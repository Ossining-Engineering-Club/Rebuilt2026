package main.java.frc.robot.subsystems.Feeder;

import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {
    @AutoLog
    public static class FeederIOInputs {
        public double appliedVolts = 0.0;
        public double statorCurrent = 0.0;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(FeederIOInputs inputs) {}

    /** Sets voltage of motor */
    public default void setFeederMotorVoltage(double voltage) {}
}