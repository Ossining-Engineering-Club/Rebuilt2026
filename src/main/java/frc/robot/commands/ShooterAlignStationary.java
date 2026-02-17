package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooterflywheels.ShooterFlywheels;
import frc.robot.subsystems.shooterhood.ShooterHood;
import frc.robot.subsystems.turret.Turret;

public class ShooterAlignStationary extends Command {
  private final Drive drive;
  private final Turret turret;
  private final ShooterFlywheels flywheels;
  private final ShooterHood hood;
}
