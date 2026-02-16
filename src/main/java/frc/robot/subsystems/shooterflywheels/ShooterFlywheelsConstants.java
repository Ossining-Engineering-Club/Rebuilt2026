package frc.robot.subsystems.shooterflywheels;

import edu.wpi.first.math.system.plant.DCMotor;

public class ShooterFlywheelsConstants {
  // motor constants
  public static final int leftMotorCanId = 0;
  public static final int rightMotorCanId = 0;
  public static final boolean isInverted = false;
  public static final double motorReduction = 8.0 / 9.0;
  public static final int supplyCurrentLimit = 38;

  // control system
  public static final double kP = 0.0;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  public static final double kS = 0.0;
  public static final double kV = 0.0;
  public static final double simP = 0.0;
  public static final double simI = 0.0;
  public static final double simD = 0.0;
  public static final double simS = 0.0;
  public static final double simV = 0.0;

  // sim settings
  public static final DCMotor gearbox = DCMotor.getKrakenX60(2);
  public static final double moi = 0.0016393466;
}
