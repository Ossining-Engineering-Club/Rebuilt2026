package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;

public final class DriveConstants {
  public static final double maxSpeedMetersPerSec = 5.24; // 4.262253; // 5.24;
  public static final double odometryFrequency = 100.0;
  public static final double trackWidth = Units.inchesToMeters(24.75); // meters
  public static final double wheelBase = Units.inchesToMeters(18.75); // meters
  public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
  public static final double wheelRadiusMeters = Units.inchesToMeters(2.0);
  public static final Translation2d[] moduleTranslations =
      new Translation2d[] {
        new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)
      };

  public static final Current slipCurrent = Amps.of(120.0);

  // Drive motor configuration
  public static final double driveMotorReduction = 5.27 / 1.0;
  public static final DCMotor driveGearbox = DCMotor.getKrakenX60(1);

  // Drive PID configuration
  public static final double driveP = 0;
  public static final double driveI = 0;
  public static final double driveD = 0;
  public static final double driveS = 0.13233;
  public static final double driveV = 0.81857;
  public static final double driveA = 0;
  public static final double driveSimP = 0.1;
  public static final double driveSimI = 0.0;
  public static final double driveSimD = 0.0;
  public static final double driveSimS = 0.07444;
  public static final double driveSimV = 0.72619;
  public static final double driveSimA = 0;

  // Turn motor configuration
  public static final int turnMotorStatorCurrentLimit = 40;
  public static final double turnMotorReduction = 287.0 / 11.0;
  public static final DCMotor turnGearbox = DCMotor.getKrakenX44(1);

  // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
  // This may need to be tuned to your individual robot
  public static final double coupleRatio = 54.0 / 16.0;

  // Turn PID configuration
  public static final double turnP = 50;
  public static final double turnI = 0;
  public static final double turnD = 0;
  public static final double turnS = 0;
  public static final double turnV = 0;
  public static final double turnA = 0;
  public static final double turnSimP = 30;
  public static final double turnSimI = 0;
  public static final double turnSimD = 0;
  public static final double turnSimS = 0;
  public static final double turnSimV = 0;
  public static final double turnSimA = 0;
  public static final double turnPIDMinInput = -Math.PI; // Radians
  public static final double turnPIDMaxInput = Math.PI; // Radians

  // These are only used for simulation
  public static final MomentOfInertia steerInertia = KilogramSquareMeters.of(0.015);
  public static final MomentOfInertia driveInertia = KilogramSquareMeters.of(0.025);
  // Simulated voltage necessary to overcome friction
  public static final Voltage steerFrictionVoltage = Volts.of(1.2);
  public static final Voltage driveFrictionVoltage = Volts.of(0.2);

  public static final int pigeonId = 13;

  public static final boolean invertLeftSide = false;
  public static final boolean invertRightSide = false;

  // Front Left
  public static final int frontLeftDriveMotorId = 1;
  public static final int frontLeftSteerMotorId = 5;
  public static final int frontLeftEncoderId = 9;
  public static final Angle frontLeftEncoderOffset = Radians.of(2.097 - Math.PI);
  public static final boolean frontLeftSteerMotorInverted = false;
  public static final boolean frontLeftEncoderInverted = false;

  public static final Distance frontLeftXPos = Meters.of(moduleTranslations[0].getX());
  public static final Distance frontLeftYPos = Meters.of(moduleTranslations[0].getY());

  // Front Right
  public static final int frontRightDriveMotorId = 2;
  public static final int frontRightSteerMotorId = 6;
  public static final int frontRightEncoderId = 10;
  public static final Angle frontRightEncoderOffset = Radians.of(0.831);
  public static final boolean frontRightSteerMotorInverted = false;
  public static final boolean frontRightEncoderInverted = false;

  public static final Distance frontRightXPos = Meters.of(moduleTranslations[1].getX());
  public static final Distance frontRightYPos = Meters.of(moduleTranslations[1].getY());

  // Back Left
  public static final int backLeftDriveMotorId = 3;
  public static final int backLeftSteerMotorId = 7;
  public static final int backLeftEncoderId = 11;
  public static final Angle backLeftEncoderOffset = Radians.of(2.178 - Math.PI);
  public static final boolean backLeftSteerMotorInverted = false;
  public static final boolean backLeftEncoderInverted = false;

  public static final Distance backLeftXPos = Meters.of(moduleTranslations[2].getX());
  public static final Distance backLeftYPos = Meters.of(moduleTranslations[2].getY());

  // Back Right
  public static final int backRightDriveMotorId = 4;
  public static final int backRightSteerMotorId = 8;
  public static final int backRightEncoderId = 12;
  public static final Angle backRightEncoderOffset = Radians.of(1.835);
  public static final boolean backRightSteerMotorInverted = false;
  public static final boolean backRightEncoderInverted = false;

  public static final Distance backRightXPos = Meters.of(moduleTranslations[3].getX());
  public static final Distance backRightYPos = Meters.of(moduleTranslations[3].getY());

  // PathPlanner configuration
  public static final double robotMassKg = Units.lbsToKilograms(130);
  public static final double robotMOI = 5.6329362863;
  public static final double wheelCOF = 2.255; // 1.2;
  public static final RobotConfig ppConfig =
      new RobotConfig(
          robotMassKg,
          robotMOI,
          new ModuleConfig(
              wheelRadiusMeters,
              maxSpeedMetersPerSec,
              wheelCOF,
              DCMotor.getKrakenX60(1).withReduction(driveMotorReduction),
              slipCurrent.magnitude(),
              1),
          moduleTranslations);
}
