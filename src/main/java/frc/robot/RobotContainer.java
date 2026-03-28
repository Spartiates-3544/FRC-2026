package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

import frc.robot.commands.RunIndexer;
import frc.robot.commands.RunIntake;
import frc.robot.commands.SetTurretAngle;
import frc.robot.commands.Shoot;
import frc.robot.commands.ShootMoving;
import frc.robot.commands.SpinUpShooter;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SpindexerSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.UnjammerSubsystem;
import frc.robot.subsystems.control.ShooterLoop;

public class RobotContainer {
        // =========================
        // Driver controller
        // =========================
        private final CommandXboxController joystick = new CommandXboxController(0);

        // =========================
        // Drive config
        // =========================
        private final double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        private final double maxAngularRate = RotationsPerSecond.of(2).in(RadiansPerSecond);

        private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
                        .withDeadband(maxSpeed * 0.1)
                        .withRotationalDeadband(maxAngularRate * 0.1)
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        private final Telemetry telemetry = new Telemetry(maxSpeed);

        // =========================
        // Subsystems
        // =========================
        public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
        public final IntakeSubsystem intake = new IntakeSubsystem();
        public final TurretSubsystem turret = new TurretSubsystem();
        public final SpindexerSubsystem spindexer = new SpindexerSubsystem();
        public final ShooterSubsystem shooter = new ShooterSubsystem();
        public final UnjammerSubsystem unjammer = new UnjammerSubsystem();
        public final ShooterLoop shooterLoop = new ShooterLoop(drivetrain, shooter, turret);

        // =========================
        // Autonomous
        // =========================
        private final SendableChooser<Command> autoChooser;

        public RobotContainer() {
                configureDefaultCommands();
                configureBindings();
                configureDashboard();

                autoChooser = AutoBuilder.buildAutoChooser();
                SmartDashboard.putData("Auto Chooser", autoChooser);

                NamedCommands.registerCommand("ramasser", new RunIntake(intake).withTimeout(3));
                NamedCommands.registerCommand("Readyshoot", buildMovingShootCommand().withTimeout(3));
        }

        // =========================
        // Setup
        // =========================
        private void configureDefaultCommands() {
                drivetrain.setDefaultCommand(
                                drivetrain.applyRequest(() -> driveRequest
                                                .withVelocityX(getDriveX())
                                                .withVelocityY(getDriveY())
                                                .withRotationalRate(getDriveOmega())));

                final var idle = new SwerveRequest.Idle();
                RobotModeTriggers.disabled().whileTrue(
                                drivetrain.applyRequest(() -> idle).ignoringDisable(true));
        }

        private void configureBindings() {
                configureDriveBindings();
                configureTurretBindings();
                configureShooterBindings();
                configureIntakeBindings();
                configureDisabledBindings();

                joystick.back().onTrue(
                                Commands.runOnce(() -> drivetrain.resetPose(new Pose2d(3, 4, Rotation2d.kZero))));
        }

        private void configureDashboard() {
                drivetrain.registerTelemetry(telemetry::telemeterize);
        }

        // =========================
        // Drive bindings
        // =========================
        private void configureDriveBindings() {
                joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        }

        // =========================
        // Turret bindings
        // =========================
        private void configureTurretBindings() {
                joystick.povLeft().onTrue(new SetTurretAngle(turret, Constants.Commands.TURRET_PRESET_LEFT_DEG));
                joystick.povRight().onTrue(new SetTurretAngle(turret, Constants.Commands.TURRET_PRESET_RIGHT_DEG));
                joystick.povDown().onTrue(new SetTurretAngle(turret, Constants.Commands.TURRET_PRESET_DOWN_DEG));
                joystick.y().toggleOnTrue(turret.home());
        }

        // =========================
        // Shooter bindings
        // =========================
        private void configureShooterBindings() {
                joystick.a().whileTrue(buildMovingShootCommand());
                joystick.b().onTrue(shooter.homeHood());
        }

        // =========================
        // Intake bindings
        // =========================
        private void configureIntakeBindings() {
                joystick.x().toggleOnTrue(new RunIntake(intake));
        }

        // =========================
        // Disabled bindings
        // =========================
        private void configureDisabledBindings() {
                RobotModeTriggers.disabled().onTrue(
                                Commands.runOnce(turret::stop, turret));
        }

        // =========================
        // Command builders
        // =========================
        private Command buildMovingShootCommand() {
                return Commands.sequence(
                                new SpinUpShooter(shooter, shooterLoop),
                                Commands.parallel(
                                                new ShootMoving(shooter, turret, shooterLoop),
                                                new RunIndexer(spindexer),
                                                new RunIntake(intake),
                                                Commands.runEnd(
                                                                () -> unjammer.setSpeed(
                                                                                Constants.Commands.UNJAMMER_SPEED),
                                                                unjammer::stop,
                                                                unjammer)));
        }

       

        // =========================
        // Driver input shaping
        // =========================
        private double getDriveX() {
                return -Math.copySign(Math.pow(joystick.getLeftY(), 2), joystick.getLeftY()) * maxSpeed;
        }

        private double getDriveY() {
                return -Math.copySign(Math.pow(joystick.getLeftX(), 2), joystick.getLeftX()) * maxSpeed;
        }
 
        private double getDriveOmega() {
                return -(joystick.getRightTriggerAxis() - joystick.getLeftTriggerAxis()) * maxAngularRate;
        }

        // =========================
        // Public hooks
        // =========================
        public Command getInitCommand() {
                return Commands.sequence(
                                turret.home(),
                                turret.setTargetAngleCommand(0.0));
        }

        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }
}