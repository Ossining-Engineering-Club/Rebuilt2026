package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
import frc.robot.subsystems.turret.TurretConstants;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class ShootOnTheMove extends Command {
  private final Drive drive;
  private final Turret turret;
  private final ShooterFlywheels shooterFlywheels;
  private final ShooterHood shooterHood;
  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;
  private final DoubleSupplier omegaSupplier;
  private InterpolatingDoubleTreeMap shooterRPMMap;
  private InterpolatingDoubleTreeMap shooterHoodMap;
  private InterpolatingDoubleTreeMap TOFMap;
  private double latencyCompensationSeconds;
  private double chassisSpeedsMultiplier;
  private double distanceIncreaseScalar;
  private int maxTOFRecursions;
  private double TOFRecursionTolerance;
  private double driveRotA;
  private final ProfiledPIDController mainDriveRotPID;
  private final PIDController secondaryDriveRotPID;
  private Translation2d target;

  private double prevSetpointVelocity = 0;

  public ShootOnTheMove(
      Drive drive,
      Turret turret,
      ShooterFlywheels shooterFlywheels,
      ShooterHood shooterHood,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    this.drive = drive;
    this.turret = turret;
    this.shooterFlywheels = shooterFlywheels;
    this.shooterHood = shooterHood;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.omegaSupplier = omegaSupplier;

    addRequirements(drive, turret, shooterFlywheels, shooterHood);

    switch (Constants.currentMode) {
      case REAL:
        mainDriveRotPID =
            new ProfiledPIDController(
                0,
                0,
                0,
                new TrapezoidProfile.Constraints(
                    ShooterAlignConstants.Real.driveRotMaxAngularVelocity,
                    ShooterAlignConstants.Real.driveRotMaxAngularAcceleration));
        secondaryDriveRotPID =
            new PIDController(
                ShooterAlignConstants.Real.driveRotP,
                ShooterAlignConstants.Real.driveRotI,
                ShooterAlignConstants.Real.driveRotD);
        break;
      case SIM:
        mainDriveRotPID =
            new ProfiledPIDController(
                0,
                0,
                0,
                new TrapezoidProfile.Constraints(
                    ShooterAlignConstants.Sim.driveRotMaxAngularVelocity,
                    ShooterAlignConstants.Sim.driveRotMaxAngularAcceleration));
        secondaryDriveRotPID =
            new PIDController(
                ShooterAlignConstants.Sim.driveRotP,
                ShooterAlignConstants.Sim.driveRotI,
                ShooterAlignConstants.Sim.driveRotD);
        break;
      case REPLAY:
        mainDriveRotPID =
            new ProfiledPIDController(
                0,
                0,
                0,
                new TrapezoidProfile.Constraints(
                    ShooterAlignConstants.Real.driveRotMaxAngularVelocity,
                    ShooterAlignConstants.Real.driveRotMaxAngularAcceleration));
        secondaryDriveRotPID =
            new PIDController(
                ShooterAlignConstants.Real.driveRotP,
                ShooterAlignConstants.Real.driveRotI,
                ShooterAlignConstants.Real.driveRotD);
        break;
      default:
        mainDriveRotPID =
            new ProfiledPIDController(
                0,
                0,
                0,
                new TrapezoidProfile.Constraints(
                    ShooterAlignConstants.Real.driveRotMaxAngularVelocity,
                    ShooterAlignConstants.Real.driveRotMaxAngularAcceleration));
        secondaryDriveRotPID =
            new PIDController(
                ShooterAlignConstants.Real.driveRotP,
                ShooterAlignConstants.Real.driveRotI,
                ShooterAlignConstants.Real.driveRotD);
        break;
    }

    mainDriveRotPID.enableContinuousInput(-Math.PI, Math.PI);
    secondaryDriveRotPID.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void initialize() {
    if (Constants.currentMode == Constants.Mode.SIM) {
      shooterRPMMap = ShooterAlignConstants.Sim.shooterRPMMap;
      shooterHoodMap = ShooterAlignConstants.Sim.shooterHoodMap;
      TOFMap = ShooterAlignConstants.Sim.TOFMap;
      latencyCompensationSeconds = ShooterAlignConstants.Sim.latencyCompensationSeconds;
      chassisSpeedsMultiplier = ShooterAlignConstants.Sim.chassisSpeedsMultiplier;
      distanceIncreaseScalar = ShooterAlignConstants.Sim.distanceIncreaseScalar;
      maxTOFRecursions = ShooterAlignConstants.Sim.maxTOFRecursions;
      TOFRecursionTolerance = ShooterAlignConstants.Sim.TOFRecursionTolerance;
      driveRotA = ShooterAlignConstants.Sim.driveRotA;
    } else {
      shooterRPMMap = ShooterAlignConstants.Real.shooterRPMMap;
      shooterHoodMap = ShooterAlignConstants.Real.shooterHoodMap;
      TOFMap = ShooterAlignConstants.Real.TOFMap;
      latencyCompensationSeconds = ShooterAlignConstants.Real.latencyCompensationSeconds;
      chassisSpeedsMultiplier = ShooterAlignConstants.Real.chassisSpeedsMultiplier;
      distanceIncreaseScalar = ShooterAlignConstants.Real.distanceIncreaseScalar;
      maxTOFRecursions = ShooterAlignConstants.Real.maxTOFRecursions;
      TOFRecursionTolerance = ShooterAlignConstants.Real.TOFRecursionTolerance;
      driveRotA = ShooterAlignConstants.Real.driveRotA;
    }

    mainDriveRotPID.reset(
        drive.getRotation().getRadians(),
        drive.getFieldRelativeChassisSpeeds().omegaRadiansPerSecond);
    prevSetpointVelocity = mainDriveRotPID.getSetpoint().velocity;
  }

  @Override
  public void execute() {
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
      if (drive.getPose().getX() <= FieldConstants.blueBumpCenterX) {
        target = FieldConstants.blueHub;
      } else {
        if (drive.getPose().getY() >= FieldConstants.fieldCenterY) {
          target = FieldConstants.bluePassingTopTarget;
        } else {
          target = FieldConstants.bluePassingBottomTarget;
        }
      }
    } else {
      if (drive.getPose().getX() >= FieldConstants.redBumpCenterX) {
        target = FieldConstants.redHub;
      } else {
        if (drive.getPose().getY() >= FieldConstants.fieldCenterY) {
          target = FieldConstants.redPassingTopTarget;
        } else {
          target = FieldConstants.redPassingBottomTarget;
        }
      }
    }

    Translation2d shooterPosition =
        new Pose3d(drive.getPose())
            .plus(new Transform3d(Constants.shooterOffset, Rotation3d.kZero))
            .getTranslation()
            .toTranslation2d();

    Translation2d modifiedTarget = calculateModifiedTarget(target, shooterPosition);

    double desiredAngle = modifiedTarget.minus(shooterPosition).getAngle().getRadians();

    double desiredTurretAngle;
    // System.out.println(
    //     (wrapAngle(desiredAngle - drive.getRotation().getRadians()))
    //         + " "
    //         + TurretConstants.minAngle
    //         + " "
    //         + TurretConstants.maxAngle);
    if (wrapAngle(desiredAngle - drive.getRotation().getRadians()) >= TurretConstants.minAngle
        && wrapAngle(desiredAngle - drive.getRotation().getRadians()) <= TurretConstants.maxAngle) {
      // System.out.println("No need to adjust drivebase");
      desiredTurretAngle = wrapAngle(desiredAngle - drive.getRotation().getRadians());
      mainDriveRotPID.reset(
          drive.getRotation().getRadians(),
          drive.getFieldRelativeChassisSpeeds().omegaRadiansPerSecond);

      // Get linear velocity
      Translation2d linearVelocity =
          DriveCommands.getLinearVelocityFromJoysticks(
              xSupplier.getAsDouble(), ySupplier.getAsDouble());

      // Apply rotation deadband
      double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DriveCommands.DEADBAND);

      // Square rotation value for more precise control
      omega = Math.copySign(omega * omega, omega);

      // Cancel rotational speed if it would cause the turret to go out of range
      if (wrapAngle(
                  desiredAngle
                      - (drive.getRotation().getRadians()
                          + omega * drive.getMaxAngularSpeedRadPerSec() * 0.02))
              < TurretConstants.minAngle
          || wrapAngle(
                  desiredAngle
                      - (drive.getRotation().getRadians()
                          + omega * drive.getMaxAngularSpeedRadPerSec() * 0.02))
              > TurretConstants.maxAngle) {
        omega = 0;
      }

      // Convert to field relative speeds & send command
      ChassisSpeeds speeds =
          new ChassisSpeeds(
              linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
              linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
              omega * drive.getMaxAngularSpeedRadPerSec());
      boolean isFlipped =
          DriverStation.getAlliance().isPresent()
              && DriverStation.getAlliance().get() == Alliance.Red;
      drive.runVelocity(
          ChassisSpeeds.fromFieldRelativeSpeeds(
              speeds,
              isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation()));
    } else {
      desiredTurretAngle = wrapAngle(desiredAngle - drive.getRotation().getRadians());
      // System.out.println(
      //     "Adjusting drivebase "
      //         + wrapAngle(desiredTurretAngle - TurretConstants.maxAngle)
      //         + " "
      //         + wrapAngle(TurretConstants.minAngle - desiredTurretAngle));
      if ((desiredTurretAngle - TurretConstants.maxAngle + 2 * Math.PI) % (2 * Math.PI)
          < (TurretConstants.minAngle - desiredTurretAngle + 2 * Math.PI) % (2 * Math.PI)) {
        desiredTurretAngle = TurretConstants.maxAngle;
      } else {
        desiredTurretAngle = TurretConstants.minAngle;
      }
      double desiredDriveRotAngle =
          Rotation2d.fromRadians(wrapAngle(desiredAngle - desiredTurretAngle)).getRadians();
      Logger.recordOutput("ShooterAlignOnTheMove/DesiredDriveRotAngle", desiredDriveRotAngle);

      // Get linear velocity
      Translation2d linearVelocity =
          DriveCommands.getLinearVelocityFromJoysticks(
              xSupplier.getAsDouble(), ySupplier.getAsDouble());

      // Calculate omega using PIDs
      mainDriveRotPID.calculate(drive.getRotation().getRadians(), desiredDriveRotAngle);
      double rotPIDOutput =
          secondaryDriveRotPID.calculate(
              drive.getRotation().getRadians(), mainDriveRotPID.getSetpoint().position);
      // System.out.println(
      //     drive.getRotation().getRadians()
      //         + " "
      //         + mainDriveRotPID.getSetpoint().position
      //         + " "
      //         + rotPIDOutput);
      double omegaPID =
          rotPIDOutput
              // + mainDriveRotPID.getSetpoint().velocity
              + driveRotA
                  * ((mainDriveRotPID.getSetpoint().velocity - prevSetpointVelocity) / 0.02);

      // Apply rotation deadband
      double omegaJoystick =
          MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DriveCommands.DEADBAND);

      // Square rotation value for more precise control and convert to rad/sec
      omegaJoystick =
          Math.copySign(omegaJoystick * omegaJoystick, omegaJoystick)
              * drive.getMaxAngularSpeedRadPerSec();

      Logger.recordOutput("rotPIDOutput", rotPIDOutput);
      Logger.recordOutput(
          "acceleration compensation",
          driveRotA * (mainDriveRotPID.getSetpoint().velocity - prevSetpointVelocity) / 0.02);
      Logger.recordOutput("rot pid setpoint velocity", mainDriveRotPID.getSetpoint().velocity);

      // If rotation from joystick and rotation from PID are in the same direction, use whichever
      // one has a greater magnitude
      // else, just use the rotation from the PID
      double omega;
      if (Math.signum(omegaJoystick) == Math.signum(omegaPID) || Math.signum(omegaPID) == 0) {
        omega = Math.copySign(Math.max(Math.abs(omegaJoystick), Math.abs(omegaPID)), omegaJoystick);
      } else {
        omega = omegaPID;
      }

      // Convert to field relative speeds & send command
      ChassisSpeeds speeds =
          new ChassisSpeeds(
              linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
              linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
              omega);
      boolean isFlipped =
          DriverStation.getAlliance().isPresent()
              && DriverStation.getAlliance().get() == Alliance.Red;
      drive.runVelocity(
          ChassisSpeeds.fromFieldRelativeSpeeds(
              speeds,
              isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation()));
    }

    double distance = modifiedTarget.minus(shooterPosition).getNorm();

    // calculating magnitude of the tangential velocity of the shooter relative to the target
    double angleFromShooterToHub = modifiedTarget.minus(shooterPosition).getAngle().getRadians();
    double shooterTangentialVelocityRelativeToTarget =
        Math.abs(
            drive.getFieldRelativeChassisSpeeds().vxMetersPerSecond
                    * Math.sin(angleFromShooterToHub)
                + drive.getFieldRelativeChassisSpeeds().vyMetersPerSecond
                    * Math.cos(angleFromShooterToHub));

    // scaling distance based on a constant and the tangential velocity of the shooter relative to
    // the target
    distance *= (1.0 + distanceIncreaseScalar * shooterTangentialVelocityRelativeToTarget);

    double desiredRPM, desiredHoodAngle;

    desiredRPM = shooterRPMMap.get(distance);
    desiredHoodAngle = shooterHoodMap.get(distance);

    Logger.recordOutput("ShooterAlignOnTheMove/DesiredTurretAngle", desiredTurretAngle);
    Logger.recordOutput("ShooterAlignOnTheMove/DesiredRPM", desiredRPM);
    Logger.recordOutput("ShooterAlignOnTheMove/DesiredHoodAngle", desiredHoodAngle);
    Logger.recordOutput("ShooterAlignOnTheMove/Distance", distance);
    Logger.recordOutput(
        "ShooterAlignOnTheMove/ShooterTangentialVelocityRelativeToTarget",
        shooterTangentialVelocityRelativeToTarget);
    Logger.recordOutput(
        "ShooterAlignOnTheMove/DriveRotPIDSetpoint", mainDriveRotPID.getSetpoint().position);

    Logger.recordOutput(
        "ShooterAlignOnTheMove/ShooterPosition", new Pose2d(shooterPosition, Rotation2d.kZero));
    Logger.recordOutput("ShooterAlignOnTheMove/Target", new Pose2d(target, Rotation2d.kZero));
    Logger.recordOutput(
        "ShooterAlignOnTheMove/ModifiedTarget", new Pose2d(modifiedTarget, Rotation2d.kZero));

    turret.runGoal(desiredTurretAngle);
    shooterFlywheels.setRPM(desiredRPM);
    shooterHood.runGoal(desiredHoodAngle);

    prevSetpointVelocity = mainDriveRotPID.getSetpoint().velocity;
  }

  @Override
  public void end(boolean interrupted) {
    turret.stop();
    shooterFlywheels.stop();
    shooterHood.stop();
  }

  public Translation2d calculateModifiedTarget(
      Translation2d target, Translation2d shooterPosition) {
    double distance = target.minus(shooterPosition).getNorm();
    double TOF = TOFMap.get(distance);
    Translation2d modifiedTarget =
        target.minus(
            new Translation2d(
                drive.getFieldRelativeChassisSpeeds().vxMetersPerSecond
                    * chassisSpeedsMultiplier
                    * (TOF + latencyCompensationSeconds),
                drive.getFieldRelativeChassisSpeeds().vyMetersPerSecond
                    * chassisSpeedsMultiplier
                    * (TOF + latencyCompensationSeconds)));

    Logger.recordOutput(
        "ShooterAlignOnTheMove/OriginalModifiedTarget",
        new Pose2d(modifiedTarget, Rotation2d.kZero));

    int numRecursions = 0;

    for (int i = 0; i < maxTOFRecursions; i++) {
      double newDistance = modifiedTarget.minus(shooterPosition).getNorm();
      double newTOF = TOFMap.get(newDistance);
      modifiedTarget =
          target.minus(
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
    return modifiedTarget;
  }

  private double wrapAngle(double angleRadians) {
    return (angleRadians + 101 * Math.PI) % (2 * Math.PI) - Math.PI;
  }
}
