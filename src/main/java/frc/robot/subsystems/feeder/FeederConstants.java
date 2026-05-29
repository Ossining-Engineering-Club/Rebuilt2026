package frc.robot.subsystems.feeder;

public class FeederConstants {
  public static final double reverseVoltage = -6.0;
  public static final double forwardVoltage = 12.0 * 0.9;
  public static final int feederCANID = 22;
  public static final boolean isInverted = false;

  // current limits for Spark Max
  public static final int feederMotorStallCurrentLimit = 38;
  public static final int feederMotorFreeCurrentLimit = 38;

  // current limit for TalonFX
  public static final int feederMotorSupplyCurrentLimit = 38;

  // current threshold for jam
  public static final double jamStatorCurrentThreshold = 80;
}
