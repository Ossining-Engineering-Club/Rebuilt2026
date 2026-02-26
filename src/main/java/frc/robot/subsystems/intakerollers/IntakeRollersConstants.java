package frc.robot.subsystems.intakerollers;

public class IntakeRollersConstants {
  // MOST VALUES OF 0.0 ARE PLACEHOLDERS
  // CAN ID
  public static final int intakeRollersCANID = 21;

  // Motor Power Level
  public static final double forwardVoltage = 0.8 * 12.0;
  public static final double reverseVoltage = -1.0;

  // Motor Constants
  public static final boolean isInverted = true;
  public static final double rollersMotorReduction = 0.0;
  public static final double encoderPositionFactor = 1.0; // Rotations -> Rotations
  public static final double encoderVelocityFactor = 1.0 / 60; // RPM -> Rot/Sec

  // current limits for Spark Max
  public static final int rollersMotorFreeCurrentLimit = 38;
  public static final int rollersMotorStallCurrentLimit = 38;

  // current limit for TalonFX
  public static final int rollersMotorSupplyCurrentLimit = 38;
}
