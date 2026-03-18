package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.ShooterAlignConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooterflywheels.ShooterFlywheels;
import frc.robot.subsystems.shooterhood.ShooterHood;
import frc.robot.subsystems.turret.Turret;
import org.littletonrobotics.junction.Logger;

public class ShootOnTheMoveAuto extends Command {
  private final Drive drive;
  private final Turret turret;
  private final ShooterFlywheels shooterFlywheels;
  private final ShooterHood shooterHood;
  private InterpolatingDoubleTreeMap shooterRPMMap;
  private InterpolatingDoubleTreeMap shooterHoodMap;
  private InterpolatingDoubleTreeMap TOFMap;
  private double latencyCompensationSeconds;
  private double chassisSpeedsMultiplier;
  private double distanceIncreaseScalar;
  private int maxTOFRecursions;
  private double TOFRecursionTolerance;
  private Translation2d targetHub;

  public ShootOnTheMoveAuto(
      Drive drive, Turret turret, ShooterFlywheels shooterFlywheels, ShooterHood shooterHood) {
    this.drive = drive;
    this.turret = turret;
    this.shooterFlywheels = shooterFlywheels;
    this.shooterHood = shooterHood;

    addRequirements(turret, shooterFlywheels, shooterHood);
  }

  @Override
  public void initialize() {
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
      targetHub = FieldConstants.blueHub;
    } else {
      targetHub = FieldConstants.redHub;
    }

    if (Constants.currentMode == Constants.Mode.SIM) {
      shooterRPMMap = ShooterAlignConstants.SimAuto.shooterRPMMap;
      shooterHoodMap = ShooterAlignConstants.SimAuto.shooterHoodMap;
      TOFMap = ShooterAlignConstants.SimAuto.TOFMap;
      latencyCompensationSeconds = ShooterAlignConstants.SimAuto.latencyCompensationSeconds;
      chassisSpeedsMultiplier = ShooterAlignConstants.SimAuto.chassisSpeedsMultiplier;
      distanceIncreaseScalar = ShooterAlignConstants.SimAuto.distanceIncreaseScalar;
      maxTOFRecursions = ShooterAlignConstants.SimAuto.maxTOFRecursions;
      TOFRecursionTolerance = ShooterAlignConstants.SimAuto.TOFRecursionTolerance;
    } else {
      shooterRPMMap = ShooterAlignConstants.RealAuto.shooterRPMMap;
      shooterHoodMap = ShooterAlignConstants.RealAuto.shooterHoodMap;
      TOFMap = ShooterAlignConstants.RealAuto.TOFMap;
      latencyCompensationSeconds = ShooterAlignConstants.RealAuto.latencyCompensationSeconds;
      chassisSpeedsMultiplier = ShooterAlignConstants.RealAuto.chassisSpeedsMultiplier;
      distanceIncreaseScalar = ShooterAlignConstants.RealAuto.distanceIncreaseScalar;
      maxTOFRecursions = ShooterAlignConstants.RealAuto.maxTOFRecursions;
      TOFRecursionTolerance = ShooterAlignConstants.RealAuto.TOFRecursionTolerance;
    }
  }

  @Override
  public void execute() {
    Translation2d shooterPosition =
        new Pose3d(drive.getPose())
            .plus(new Transform3d(Constants.shooterOffset, Rotation3d.kZero))
            .getTranslation()
            .toTranslation2d();

    Translation2d modifiedHub = calculateModifiedHub(targetHub, shooterPosition);

    double desiredTurretAngle =
        modifiedHub.minus(shooterPosition).getAngle().getRadians()
            - drive.getRotation().getRadians();

    double distance = modifiedHub.minus(shooterPosition).getNorm();

    // calculating magnitude of the tangential velocity of the shooter relative to the hub
    double angleFromShooterToHub = modifiedHub.minus(shooterPosition).getAngle().getRadians();
    double shooterTangentialVelocityRelativeToHub =
        Math.abs(
            drive.getFieldRelativeChassisSpeeds().vxMetersPerSecond
                    * Math.sin(angleFromShooterToHub)
                + drive.getFieldRelativeChassisSpeeds().vyMetersPerSecond
                    * Math.cos(angleFromShooterToHub));

    // scaling distance based on a constant and the tangential velocity of the shooter relative to
    // the
    // hub
    distance *= (1.0 + distanceIncreaseScalar * shooterTangentialVelocityRelativeToHub);

    double desiredRPM, desiredHoodAngle;

    desiredRPM = shooterRPMMap.get(distance);
    desiredHoodAngle = shooterHoodMap.get(distance);

    Logger.recordOutput("ShooterAlignOnTheMove/DesiredTurretAngle", desiredTurretAngle);
    Logger.recordOutput("ShooterAlignOnTheMove/DesiredRPM", desiredRPM);
    Logger.recordOutput("ShooterAlignOnTheMove/DesiredHoodAngle", desiredHoodAngle);
    Logger.recordOutput("ShooterAlignOnTheMove/Distance", distance);
    Logger.recordOutput(
        "ShooterAlignOnTheMove/ShooterTangentialVelocityRelativeToHub",
        shooterTangentialVelocityRelativeToHub);

    Logger.recordOutput(
        "ShooterAlignOnTheMove/ShooterPosition", new Pose2d(shooterPosition, Rotation2d.kZero));
    Logger.recordOutput("ShooterAlignOnTheMove/TargetHub", new Pose2d(targetHub, Rotation2d.kZero));
    Logger.recordOutput(
        "ShooterAlignOnTheMove/ModifiedHub", new Pose2d(modifiedHub, Rotation2d.kZero));

    turret.runGoal(desiredTurretAngle);
    shooterFlywheels.setRPM(desiredRPM);
    shooterHood.runGoal(desiredHoodAngle);
  }

  @Override
  public void end(boolean interrupted) {
    turret.stop();
    shooterFlywheels.stop();
    shooterHood.stop();
  }

  public Translation2d calculateModifiedHub(
      Translation2d targetHub, Translation2d shooterPosition) {
    double distance = targetHub.minus(shooterPosition).getNorm();
    double TOF = TOFMap.get(distance);
    Translation2d modifiedHub =
        targetHub.minus(
            new Translation2d(
                drive.getFieldRelativeChassisSpeeds().vxMetersPerSecond
                    * chassisSpeedsMultiplier
                    * (TOF + latencyCompensationSeconds),
                drive.getFieldRelativeChassisSpeeds().vyMetersPerSecond
                    * chassisSpeedsMultiplier
                    * (TOF + latencyCompensationSeconds)));

    Logger.recordOutput(
        "ShooterAlignOnTheMove/OriginalModifiedHub", new Pose2d(modifiedHub, Rotation2d.kZero));

    int numRecursions = 0;

    for (int i = 0; i < maxTOFRecursions; i++) {
      double newDistance = modifiedHub.minus(shooterPosition).getNorm();
      double newTOF = TOFMap.get(newDistance);
      modifiedHub =
          targetHub.minus(
              new Translation2d(
                  drive.getFieldRelativeChassisSpeeds().vxMetersPerSecond
                      * chassisSpeedsMultiplier
                      * (newTOF + latencyCompensationSeconds),
                  drive.getFieldRelativeChassisSpeeds().vyMetersPerSecond
                      * chassisSpeedsMultiplier
                      * (newTOF + latencyCompensationSeconds)));
      numRecursions++;
      if (Math.abs(newTOF - TOF) / TOF <= TOFRecursionTolerance) {
        TOF = newTOF;
        break;
      }
      TOF = newTOF;
    }
    Logger.recordOutput("ShooterAlignOnTheMove/EstimatedTOF", TOF);
    Logger.recordOutput("ShooterAlignOnTheMove/TOFRecursions", numRecursions);
    return modifiedHub;
  }
}
