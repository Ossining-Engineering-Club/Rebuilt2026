package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.ShooterAlignConstants;
import frc.robot.subsystems.shooterflywheels.ShooterFlywheels;
import frc.robot.subsystems.shooterhood.ShooterHood;
import frc.robot.subsystems.turret.Turret;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class ShooterAlignStationary extends Command {
  private final Supplier<Pose2d> robotPoseSupplier;
  private final Turret turret;
  private final ShooterFlywheels shooterFlywheels;
  private final ShooterHood shooterHood;
  private Translation2d targetHub;

  public ShooterAlignStationary(
      Supplier<Pose2d> robotPoseSupplier,
      Turret turret,
      ShooterFlywheels shooterFlywheels,
      ShooterHood shooterHood) {
    this.robotPoseSupplier = robotPoseSupplier;
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
  }

  @Override
  public void execute() {
    Translation2d shooterPosition =
        new Pose3d(robotPoseSupplier.get())
            .plus(new Transform3d(Constants.shooterOffset, Rotation3d.kZero))
            .getTranslation()
            .toTranslation2d();

    double desiredTurretAngle =
        targetHub.minus(shooterPosition).getAngle().getRadians()
            - robotPoseSupplier.get().getRotation().getRadians();
    double distance = targetHub.minus(shooterPosition).getNorm();
    double desiredRPM, desiredHoodAngle;

    if (Constants.currentMode == Constants.Mode.SIM) {
      desiredRPM = ShooterAlignConstants.Sim.shooterRPMMap.get(distance);
      desiredHoodAngle = ShooterAlignConstants.Sim.shooterHoodMap.get(distance);
    } else {
      desiredRPM = ShooterAlignConstants.Real.shooterRPMMap.get(distance);
      desiredHoodAngle = ShooterAlignConstants.Real.shooterHoodMap.get(distance);
    }

    Logger.recordOutput("ShooterAlignStationary/DesiredTurretAngle", desiredTurretAngle);
    Logger.recordOutput("ShooterAlignStationary/DesiredRPM", desiredRPM);
    Logger.recordOutput("ShooterAlignStationary/DesiredHoodAngle", desiredHoodAngle);
    Logger.recordOutput("ShooterAlignStationary/Distance", distance);

    Logger.recordOutput(
        "ShooterAlignStationary/ShooterPosition", new Pose2d(shooterPosition, Rotation2d.kZero));
    Logger.recordOutput(
        "ShooterAlignStationary/TargetHub", new Pose2d(targetHub, Rotation2d.kZero));

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
}
