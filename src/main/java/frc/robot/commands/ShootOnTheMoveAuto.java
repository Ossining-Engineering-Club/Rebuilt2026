package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
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
    if (Constants.currentMode == Constants.Mode.SIM) {
      shooterRPMMap = ShooterAlignConstants.SimAuto.shooterRPMMap;
      shooterHoodMap = ShooterAlignConstants.Sim.shooterHoodMap;
      TOFMap = ShooterAlignConstants.Sim.TOFMap;
      latencyCompensationSeconds = ShooterAlignConstants.Sim.latencyCompensationSeconds;
      chassisSpeedsMultiplier = ShooterAlignConstants.Sim.chassisSpeedsMultiplier;
      distanceIncreaseScalar = ShooterAlignConstants.Sim.distanceIncreaseScalar;
      maxTOFRecursions = ShooterAlignConstants.Sim.maxTOFRecursions;
      TOFRecursionTolerance = ShooterAlignConstants.Sim.TOFRecursionTolerance;
    } else {
      shooterRPMMap = ShooterAlignConstants.Real.shooterRPMMap;
      shooterHoodMap = ShooterAlignConstants.Real.shooterHoodMap;
      TOFMap = ShooterAlignConstants.Real.TOFMap;
      latencyCompensationSeconds = ShooterAlignConstants.Real.latencyCompensationSeconds;
      chassisSpeedsMultiplier = ShooterAlignConstants.Real.chassisSpeedsMultiplier;
      distanceIncreaseScalar = ShooterAlignConstants.Real.distanceIncreaseScalar;
      maxTOFRecursions = ShooterAlignConstants.Real.maxTOFRecursions;
      TOFRecursionTolerance = ShooterAlignConstants.Real.TOFRecursionTolerance;
    }
  }

  @Override
  public void execute() {
    Translation2d targetHub;
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
      targetHub = FieldConstants.blueHub;
    } else {
      targetHub = FieldConstants.redHub;
    }

    Translation2d shooterPosition =
        drive
            .getPose()
            .plus(new Transform2d(Constants.shooterOffset.toTranslation2d(), Rotation2d.kZero))
            .getTranslation();

    Translation2d modifiedHub = calculateModifiedHub(targetHub, shooterPosition);

    double desiredTurretAngle =
        modifiedHub.minus(shooterPosition).getAngle().getRadians()
            - drive.getRotation().getRadians();

    double distance = modifiedHub.minus(shooterPosition).getNorm();

    // calculating magnitude of the radial velocity of the shooter relative to the hub
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

    double newTOF = 0;
    Translation2d newModifiedHub = new Translation2d();

    for (int i = 0; i < maxTOFRecursions; i++) {
      double newDistance = modifiedHub.minus(shooterPosition).getNorm();
      newTOF = TOFMap.get(newDistance);
      newModifiedHub =
          modifiedHub.minus(
              new Translation2d(
                  drive.getFieldRelativeChassisSpeeds().vxMetersPerSecond
                      * chassisSpeedsMultiplier
                      * (newTOF + latencyCompensationSeconds),
                  drive.getFieldRelativeChassisSpeeds().vyMetersPerSecond
                      * chassisSpeedsMultiplier
                      * (newTOF + latencyCompensationSeconds)));
      if (Math.abs(newTOF - TOF) / TOF <= TOFRecursionTolerance) break;
    }
    Logger.recordOutput("ShooterAlignOnTheMove/EstimatedTOF", newTOF);
    return newModifiedHub;
  }
}
