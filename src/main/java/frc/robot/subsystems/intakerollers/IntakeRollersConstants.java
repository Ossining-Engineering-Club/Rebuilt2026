package frc.robot.subsystems.intakerollers;

import edu.wpi.first.math.util.Units;

public class IntakeRollersConstants {
  // CAN ID
  public static final int intakeRollersLeftCANID = 24;
  public static final int intakeRollersRightCANID = 21;

  // Motor Power Level
  public static final double forwardVoltage = 0.8 * 12.0;
  public static final double reverseVoltage = -0.5 * 12.0;

  // Motor Constants
  public static final boolean isInverted = true;
  public static final double rollersMotorReduction = 0.0;
  public static final double encoderPositionFactor = 1.0; // Rotations -> Rotations
  public static final double encoderVelocityFactor = 1.0 / 60; // RPM -> Rot/Sec

  public static final double maxPivotAngleForMoving = Units.degreesToRadians(45);

  // current limits for Spark Max
  public static final int rollersMotorFreeCurrentLimit = 38;
  public static final int rollersMotorStallCurrentLimit = 38;

  // current limit for TalonFX
  public static final int rollersMotorSupplyCurrentLimit = 38;
}
