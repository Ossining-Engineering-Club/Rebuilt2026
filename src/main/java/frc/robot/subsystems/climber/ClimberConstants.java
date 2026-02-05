package frc.robot.subsystems.climber;

import edu.wpi.first.math.system.plant.DCMotor;

public class ClimberConstants {
  // 0.0 = placeholder
  public static final int climberMotorCanId = 0;

  public static final double startPosition = 0.0;
  public static final boolean isInverted = false;

  public static final double climberMotorReduction = 0.0;
  public static final double encoderPositionFactor = 1.0; // Rotations -> Rotations
  public static final double encoderVelocityFactor = 1.0 / 60.0; // RPM -> Rot/Sec

  public static final int climberMotorStallCurrentLimit = 38;
  public static final int climberMotorFreeCurrentLimit = 38;
  public static final int currentLimit = 0;

  public static final double climberMotorForwardVoltage = 12.0;
  public static final double climberMotorReverseVoltage = -12.0;

  public static final double minPosition = 0.0;
  public static final double maxPosition = 0.0;

  public static final double retractPosition = 0.0;
  public static final double extendPosition = 0.0;
  public static final double storePosition = 0.0;

  public static final double climberMOI = 0.0;

  public static final DCMotor gearbox = DCMotor.getFalcon500(1);
}
