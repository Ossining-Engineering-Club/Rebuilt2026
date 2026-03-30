package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.IntakeAgitate;
import frc.robot.commands.MaintainIntakePivotAngle;
import frc.robot.commands.ShootOnTheMove;
import frc.robot.commands.ShootOnTheMoveAuto;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeonIMU;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.FeederConstants;
import frc.robot.subsystems.feeder.FeederIO;
import frc.robot.subsystems.feeder.FeederIOSim;
import frc.robot.subsystems.feeder.FeederIOTalonFX;
import frc.robot.subsystems.intakepivot.IntakePivot;
import frc.robot.subsystems.intakepivot.IntakePivotConstants;
import frc.robot.subsystems.intakepivot.IntakePivotIO;
import frc.robot.subsystems.intakepivot.IntakePivotIOSim;
import frc.robot.subsystems.intakepivot.IntakePivotIOTalonFX;
import frc.robot.subsystems.intakerollers.IntakeRollers;
import frc.robot.subsystems.intakerollers.IntakeRollersIO;
import frc.robot.subsystems.intakerollers.IntakeRollersIOSim;
import frc.robot.subsystems.intakerollers.IntakeRollersIOTalonFX;
import frc.robot.subsystems.shooterflywheels.ShooterFlywheels;
import frc.robot.subsystems.shooterflywheels.ShooterFlywheelsIO;
import frc.robot.subsystems.shooterflywheels.ShooterFlywheelsIOReal;
import frc.robot.subsystems.shooterflywheels.ShooterFlywheelsIOSim;
import frc.robot.subsystems.shooterhood.ShooterHood;
import frc.robot.subsystems.shooterhood.ShooterHoodConstants;
import frc.robot.subsystems.shooterhood.ShooterHoodIO;
import frc.robot.subsystems.shooterhood.ShooterHoodIOReal;
import frc.robot.subsystems.shooterhood.ShooterHoodIOSim;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.spindexer.SpindexerIO;
import frc.robot.subsystems.spindexer.SpindexerIOReal;
import frc.robot.subsystems.spindexer.SpindexerIOSim;
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
  private final ShooterFlywheels shooterFlywheels;
  private final IntakeRollers intakeRollers;
  private final Spindexer spindexer;
  private final Feeder feeder;

  // Controller
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);
  private final CommandXboxController manualController = new CommandXboxController(2);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  // Sim objects
  private SwerveDriveSimulation driveSimulation = null;
  private FuelSim fuelSim = null;
  private SimulationManager simulationManager = null;

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
        intakePivot = new IntakePivot(new IntakePivotIOTalonFX());
        turret = new Turret(new TurretIOReal());
        shooterHood = new ShooterHood(new ShooterHoodIOReal());
        shooterFlywheels = new ShooterFlywheels(new ShooterFlywheelsIOReal());
        intakeRollers =
            new IntakeRollers(new IntakeRollersIOTalonFX(), () -> intakePivot.getAngle());
        spindexer = new Spindexer(new SpindexerIOReal());
        feeder = new Feeder(new FeederIOTalonFX());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        driveSimulation =
            new SwerveDriveSimulation(Drive.mapleSimConfig, new Pose2d(3, 3, new Rotation2d()));
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
        fuelSim = new FuelSim();

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
        shooterFlywheels = new ShooterFlywheels(new ShooterFlywheelsIOSim());
        intakeRollers = new IntakeRollers(new IntakeRollersIOSim(), () -> intakePivot.getAngle());
        spindexer = new Spindexer(new SpindexerIOSim());
        feeder = new Feeder(new FeederIOSim());

        simulationManager =
            new SimulationManager(
                fuelSim,
                shooterFlywheels,
                shooterHood,
                intakePivot,
                intakeRollers,
                spindexer,
                turret,
                feeder);
        configureFuelSim();
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
        shooterFlywheels = new ShooterFlywheels(new ShooterFlywheelsIO() {});
        intakeRollers = new IntakeRollers(new IntakeRollersIO() {}, () -> intakePivot.getAngle());
        spindexer = new Spindexer(new SpindexerIO() {});
        feeder = new Feeder(new FeederIO() {});
        break;
    }

    // Configure the PathPlanner Named Commands
    configureNamedCommands();

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    // autoChooser.addOption(
    //     "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Forward)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Reverse)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  private void configureNamedCommands() {
    NamedCommands.registerCommand("Retract Intake", intakePivot.retract());
    NamedCommands.registerCommand("Extend Intake", intakePivot.extend());
    // NamedCommands.registerCommand("Upright Intake", intakePivot.upright());
    NamedCommands.registerCommand(
        "Start Intake", Commands.runOnce(() -> intakeRollers.startMotor(), intakeRollers));
    NamedCommands.registerCommand(
        "Stop Intake", Commands.runOnce(() -> intakeRollers.stopMotor(), intakeRollers));
    NamedCommands.registerCommand(
        "Aim On The Move", new ShootOnTheMoveAuto(drive, turret, shooterFlywheels, shooterHood));
    NamedCommands.registerCommand(
        "Start Shooting",
        Commands.runOnce(
            () -> {
              feeder.startMotor();
              spindexer.startMotor();
            },
            feeder,
            spindexer));
    NamedCommands.registerCommand(
        "Stop Shooting",
        Commands.runOnce(
            () -> {
              feeder.stopMotor();
              spindexer.stopMotor();
            },
            feeder,
            spindexer));
    NamedCommands.registerCommand(
        "Stop Flywheels", Commands.runOnce(() -> shooterFlywheels.stop(), shooterFlywheels));
    NamedCommands.registerCommand(
        "Stop Feeder", Commands.runOnce(() -> feeder.stopMotor(), feeder));
    NamedCommands.registerCommand(
        "Stop Spindexer", Commands.runOnce(() -> spindexer.stopMotor(), spindexer));
    NamedCommands.registerCommand("Intake Agitate", new IntakeAgitate(intakePivot));
    NamedCommands.registerCommand(
        "Maintain Intaking Angle",
        new MaintainIntakePivotAngle(intakePivot, IntakePivotConstants.extendedAngle));
    NamedCommands.registerCommand(
        "Store Shooter Hood", shooterHood.goToAngle(ShooterHoodConstants.startAngle));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Driver Controller
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX()));

    // Switch to X pattern when X button is pressed
    // driverController.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when A button is pressed
    driverController
        .a()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    // driverController.x().onTrue(turret.goToAngle(TurretConstants.minAngle));
    // driverController.y().onTrue(turret.goToAngle(TurretConstants.storedAngle));
    // driverController.b().onTrue(turret.goToAngle(TurretConstants.maxAngle));

    // driverController.x().onTrue(Commands.runOnce(() -> turret.setVoltage(0.1)));
    // driverController.x().onFalse(Commands.runOnce(() -> turret.stop()));

    // if (Constants.currentMode == Mode.SIM) {
    //   driverController.start().onTrue(Commands.runOnce(() -> fuelSim.clearFuel()));
    // }

    // Operator Controller
    operatorController
        .rightTrigger(0.9)
        .onTrue(
            Commands.runOnce(
                () -> {
                  feeder.startMotor();
                  spindexer.startMotor();
                },
                feeder,
                spindexer));

    operatorController
        .rightTrigger(0.9)
        .onFalse(
            Commands.runOnce(
                () -> {
                  feeder.stopMotor();
                  spindexer.stopMotor();
                },
                feeder,
                spindexer));

    operatorController
        .leftTrigger(0.9)
        .whileTrue(
            new ShootOnTheMove(
                drive,
                turret,
                shooterFlywheels,
                shooterHood,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> -driverController.getRightX()));

    operatorController
        .leftTrigger(0.9)
        .onFalse(shooterHood.goToAngle(ShooterHoodConstants.startAngle));

    operatorController
        .leftBumper()
        .whileTrue(new MaintainIntakePivotAngle(intakePivot, IntakePivotConstants.extendedAngle));

    operatorController
        .rightBumper()
        .onTrue(
            Commands.runOnce(
                () -> {
                  feeder.reverseMotor();
                  spindexer.reverseMotor();
                },
                feeder,
                spindexer));

    operatorController
        .rightBumper()
        .onFalse(
            Commands.runOnce(
                () -> {
                  feeder.stopMotor();
                  spindexer.stopMotor();
                },
                feeder,
                spindexer));

    operatorController
        .x()
        .onTrue(Commands.runOnce(() -> intakeRollers.startMotor(), intakeRollers));
    operatorController.y().onTrue(Commands.runOnce(() -> intakeRollers.stopMotor(), intakeRollers));

    // operatorController.y().onTrue(turret.goToAngle(Units.degreesToRadians(-25)));
    // operatorController.x().onTrue(turret.goToAngle(Units.degreesToRadians(0)));
    // operatorController.a().onTrue(turret.goToAngle(Units.degreesToRadians(113)));

    // operatorController.x().onTrue(Commands.runOnce(() -> turret.setVoltage(0.145), turret));
    // operatorController.x().onFalse(Commands.runOnce(() -> turret.setVoltage(0.0), turret));

    operatorController.povUp().onTrue(intakePivot.extend());
    // operatorController.povLeft().onTrue(intakePivot.upright());
    operatorController.povLeft().whileTrue(new IntakeAgitate(intakePivot));
    operatorController.povDown().onTrue(intakePivot.retract());

    operatorController
        .b()
        .onTrue(
            Commands.runOnce(
                () -> stopEverything(),
                shooterFlywheels,
                feeder,
                spindexer,
                intakePivot,
                intakeRollers,
                turret,
                shooterHood));

    // shooterFlywheels.setDefaultCommand(
    //     Commands.runOnce(
    //         () ->
    //             shooterFlywheels.setRPM(
    //                 shooterFlywheels.getRPMSetpoint()
    //                     + 10 * (-MathUtil.applyDeadband(operatorController.getRightY(), 0.1))),
    //         shooterFlywheels));

    // Manual Controller
    // Feeder manual control
    manualController
        .leftBumper()
        .onTrue(
            Commands.runOnce(
                () -> {
                  feeder.startMotor();
                },
                feeder));
    manualController
        .leftBumper()
        .onFalse(
            Commands.runOnce(
                () -> {
                  feeder.stopMotor();
                },
                feeder));

    manualController
        .rightBumper()
        .onTrue(
            Commands.runOnce(
                () -> {
                  feeder.reverseMotor();
                },
                feeder));
    manualController
        .rightBumper()
        .onFalse(
            Commands.runOnce(
                () -> {
                  feeder.stopMotor();
                },
                feeder));

    // Spindexer manual control
    manualController
        .leftTrigger(0.9)
        .onTrue(
            Commands.runOnce(
                () -> {
                  spindexer.startMotor();
                },
                spindexer));
    manualController
        .leftTrigger(0.9)
        .onFalse(
            Commands.runOnce(
                () -> {
                  spindexer.stopMotor();
                },
                spindexer));

    manualController
        .rightTrigger(0.9)
        .onTrue(
            Commands.runOnce(
                () -> {
                  spindexer.reverseMotor();
                },
                spindexer));
    manualController
        .rightTrigger(0.9)
        .onFalse(
            Commands.runOnce(
                () -> {
                  spindexer.stopMotor();
                },
                spindexer));

    // Intake Rollers manual control
    manualController
        .x()
        .onTrue(
            Commands.runOnce(
                () -> {
                  intakeRollers.startMotor();
                },
                intakeRollers));
    manualController
        .x()
        .onFalse(
            Commands.runOnce(
                () -> {
                  intakeRollers.stopMotor();
                },
                intakeRollers));

    manualController
        .y()
        .onTrue(
            Commands.runOnce(
                () -> {
                  intakeRollers.reverseMotor();
                },
                intakeRollers));
    manualController
        .y()
        .onFalse(
            Commands.runOnce(
                () -> {
                  intakeRollers.stopMotor();
                },
                intakeRollers));

    // Flywheels manual control
    manualController
        .a()
        .onTrue(Commands.runOnce(() -> shooterFlywheels.setRPM(2000), shooterFlywheels));
    manualController.a().onFalse(Commands.runOnce(() -> shooterFlywheels.stop(), shooterFlywheels));

    // Shooter Hood manual control
    manualController
        .povUp()
        .onTrue(Commands.runOnce(() -> shooterHood.setVoltage(-0.1 * 12.0), shooterHood));
    manualController
        .povUp()
        .onFalse(Commands.runOnce(() -> shooterHood.setVoltage(0), shooterHood));
    manualController
        .povDown()
        .onTrue(Commands.runOnce(() -> shooterHood.setVoltage(0.1 * 12.0), shooterHood));
    manualController
        .povDown()
        .onFalse(Commands.runOnce(() -> shooterHood.setVoltage(0), shooterHood));

    intakePivot.setDefaultCommand(
        Commands.run(
            () ->
                intakePivot.setVoltage(
                    0.5 * 12.0 * MathUtil.applyDeadband(-manualController.getLeftY(), 0.1)),
            intakePivot));

    turret.setDefaultCommand(
        Commands.run(
            () ->
                turret.setVoltage(
                    0.25 * 12.0 * MathUtil.applyDeadband(-manualController.getRightX(), 0.1)),
            turret));

    // Feeder auto unjam
    Logger.recordOutput("Time of Last Feeder Jam", -1);
    new Trigger(() -> feeder.getStatorCurrent() >= FeederConstants.jamStatorCurrentThreshold)
        .onTrue(
            Commands.runOnce(
                    () -> Logger.recordOutput("Time of Last Feeder Jam", Timer.getFPGATimestamp()))
                .andThen(Commands.runOnce(() -> feeder.reverseMotor(), feeder))
                .andThen(Commands.waitSeconds(1.0))
                .andThen(Commands.runOnce(() -> feeder.stopMotor(), feeder)));
  }

  private void configureFuelSim() {
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
        simulationManager::isIntaking,
        simulationManager::incrementFuelCount);

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
    // return Commands.runOnce(() -> {});
  }

  public void resetSimState() {
    fuelSim.clearFuel();
    fuelSim.spawnStartingFuel();
    intakePivot.resetSimState();
    shooterHood.resetSimState();
    turret.resetSimState();
    simulationManager.resetFuelForAuto();
  }

  public void stopEverything() {
    shooterFlywheels.stop();
    feeder.stopMotor();
    spindexer.stopMotor();
    intakePivot.stop();
    intakeRollers.stopMotor();
    turret.stop();
    shooterHood.stop();
  }

  public void updateSimulation() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    SimulatedArena.getInstance().simulationPeriodic();
    simulationManager.periodic();
    fuelSim.updateSim();

    Logger.recordOutput(
        "FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());

    Logger.recordOutput(
        "FieldSimulation/ChassisSpeedsFieldRelative",
        driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative());
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
