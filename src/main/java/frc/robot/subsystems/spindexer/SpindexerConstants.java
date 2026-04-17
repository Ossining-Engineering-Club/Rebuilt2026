package frc.robot.subsystems.spindexer;

public class SpindexerConstants {
  public static final int spindexerCanID = 23;

  public static final double forwardVoltage = 12.0 * 0.85;
  public static final double reverseVoltage = -6.0;

  public static final boolean isInverted = false;
  public static final double spindexerMotorReduction = 9.0 / 1.0;
  public static final int spindexerMotorSupplyCurrentLimit = 38;

  // auto jam detection
  public static final int jamSupplyCurrentThreshold = 32;
  public static final int jamTicksThreshold = 50;
}
