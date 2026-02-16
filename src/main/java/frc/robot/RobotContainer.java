package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeonIMU;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.intakepivot.IntakePivot;
import frc.robot.subsystems.intakepivot.IntakePivotIO;
import frc.robot.subsystems.intakepivot.IntakePivotIOReal;
import frc.robot.subsystems.intakepivot.IntakePivotIOSim;
import frc.robot.subsystems.shooterhood.ShooterHood;
import frc.robot.subsystems.shooterhood.ShooterHoodIO;
import frc.robot.subsystems.shooterhood.ShooterHoodIOReal;
import frc.robot.subsystems.shooterhood.ShooterHoodIOSim;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretIO;
import frc.robot.subsystems.turret.TurretIOReal;
import frc.robot.subsystems.turret.TurretIOSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOReal;
import frc.robot.subsystems.vision.VisionIOSim;
import frc.robot.util.FuelSim;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Vision vision;
  private final IntakePivot intakePivot;
  private final Turret turret;
  private final ShooterHood shooterHood;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  // Sim objects
  private SwerveDriveSimulation driveSimulation = null;
  private FuelSim fuelSim = null;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        vision =
            new Vision(
                new VisionIOReal(VisionConstants.limelightNames[0]),
                new VisionIOReal(VisionConstants.limelightNames[1]));
        drive =
            new Drive(
                new GyroIOPigeonIMU(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight),
                vision,
                (robotPose) -> {});
        intakePivot = new IntakePivot(new IntakePivotIOReal());
        turret = new Turret(new TurretIOReal());
        shooterHood = new ShooterHood(new ShooterHoodIOReal());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        driveSimulation =
            new SwerveDriveSimulation(Drive.mapleSimConfig, new Pose2d(3, 3, new Rotation2d()));
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
        configureFuelSim();
        vision =
            new Vision(
                new VisionIOSim(
                    VisionConstants.limelightNames[0],
                    VisionConstants.frontLLRobotToCamOffset,
                    driveSimulation::getSimulatedDriveTrainPose),
                new VisionIOSim(
                    VisionConstants.limelightNames[1],
                    VisionConstants.leftLLRobotToCamOffset,
                    driveSimulation::getSimulatedDriveTrainPose));
        drive =
            new Drive(
                new GyroIOSim(driveSimulation.getGyroSimulation()),
                new ModuleIOSim(TunerConstants.FrontLeft, driveSimulation.getModules()[0]),
                new ModuleIOSim(TunerConstants.FrontRight, driveSimulation.getModules()[1]),
                new ModuleIOSim(TunerConstants.BackLeft, driveSimulation.getModules()[2]),
                new ModuleIOSim(TunerConstants.BackRight, driveSimulation.getModules()[3]),
                vision,
                driveSimulation::setSimulationWorldPose);
        intakePivot = new IntakePivot(new IntakePivotIOSim());
        turret = new Turret(new TurretIOSim());
        shooterHood = new ShooterHood(new ShooterHoodIOSim());
        break;

      default:
        // Replayed robot, disable IO implementations
        vision =
            new Vision(new VisionIO() {}, new VisionIO() {}, new VisionIO() {}, new VisionIO() {});
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                vision,
                (robotPose) -> {});
        intakePivot = new IntakePivot(new IntakePivotIO() {});
        turret = new Turret(new TurretIO() {});
        shooterHood = new ShooterHood(new ShooterHoodIO() {});
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Switch to X pattern when X button is pressed
    // controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when A button is pressed
    // controller
    //     .a()
    //     .onTrue(
    //         Commands.runOnce(
    //                 () ->
    //                     drive.setPose(
    //                         new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
    //                 drive)
    //             .ignoringDisable(true));

    // controller.x().onTrue(intakePivot.goToAngle(IntakePivotConstants.extendedAngle));
    // controller.y().onTrue(intakePivot.goToAngle(IntakePivotConstants.retractedAngle));
    // controller.b().whileTrue(new IntakeAgitate(intakePivot));

    // controller.x().onTrue(turret.goToAngle(Units.degreesToRadians(150)));
    // controller.y().onTrue(turret.goToAngle(Units.degreesToRadians(-30)));
    // controller.b().whileTrue(turret.trackAngle(() -> -drive.getRotation().getRadians()));

    controller.x().onTrue(shooterHood.goToAngle(Units.degreesToRadians(40)));
    controller.y().onTrue(shooterHood.goToAngle(Units.degreesToRadians(66)));
    controller.b().whileTrue(shooterHood.trackAngle(() -> drive.getRotation().getRadians()));
  }

  private void configureFuelSim() {
    fuelSim = new FuelSim();
    fuelSim.spawnStartingFuel();

    fuelSim.registerRobot(
        Units.inchesToMeters(3 + 30 + 3),
        Units.inchesToMeters(3 + 24 + 3),
        Units.inchesToMeters(6.75),
        driveSimulation::getSimulatedDriveTrainPose,
        driveSimulation::getDriveTrainSimulatedChassisSpeedsFieldRelative);

    fuelSim.registerIntake(
        Units.inchesToMeters(-21.27965),
        Units.inchesToMeters(-15),
        Units.inchesToMeters(-12.75),
        Units.inchesToMeters(13.0625),
        () -> true);

    fuelSim.enableAirResistance();

    fuelSim.start();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void resetSimState() {
    fuelSim.clearFuel();
    fuelSim.spawnStartingFuel();
  }

  public void updateSimulation() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    SimulatedArena.getInstance().simulationPeriodic();
    fuelSim.updateSim();

    Logger.recordOutput(
        "FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
  }

  public void updateMechanismVisualization() {
    Logger.recordOutput(
        "Component Poses",
        new Pose3d[] {
          new Pose3d(
              0.130175,
              0.2032,
              0.4468150068,
              new Rotation3d(0, 0, turret.getAngle())), // Shooter Base
          new Pose3d(
              0.130175 + 0.1118757224 * Math.cos(turret.getAngle()),
              0.2032 + 0.1118757224 * Math.sin(turret.getAngle()),
              0.5103150068,
              new Rotation3d(
                  0,
                  (Math.PI / 2 - shooterHood.getAngle()) - Units.degreesToRadians(24),
                  turret.getAngle())), // Shooter Hood
          new Pose3d(
              -0.254, 0, 0.2286, new Rotation3d(0, intakePivot.getAngle(), 0)), // Intake Pivot
          new Pose3d(
              Math.max(
                  -0.284582 * Math.cos(intakePivot.getAngle() - Units.degreesToRadians(1.5343415))
                      + 0.2115,
                  0),
              0,
              0,
              new Rotation3d()) // Hopper Extension
        });
  }
}
