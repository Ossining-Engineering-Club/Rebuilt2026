package frc.robot.subsystems.feeder;

// 0.0 and 0 is a placeholder
public class FeederConstants {
  public static final double reverseVoltage = 0.0;
  public static final double forwardVoltage = 0.0;
  public static final int feederCANID = 0;
  public static final boolean isInverted = false;

  // current limits for Spark Max
  public static final int feederMotorStallCurrentLimit = 38;
  public static final int feederMotorFreeCurrentLimit = 38;

  // current limit for TalonFX
  public static final int feederMotorSupplyCurrentLimit = 38;
}
