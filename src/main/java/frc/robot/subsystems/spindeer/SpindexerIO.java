package frc.robot.subsystems.spindeer;
import org.littletonrobotics.junction.AutoLog;

public interface SpindexerIO {
    @AutoLog

    public static class SpindexerIOInputs {
        public double appliedVolts = 0.0;
        public double statorCurrent = 0.0;
    }
    

    public default void updateInputs(SpindexerIOInputs inputs) {}

    public default void setSpindexerMotorVoltage(double voltage) {}
}
