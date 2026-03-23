package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.lib.robot.Records;
import frc.robot.Constants.Drive;
import frc.robot.commands.Ramasser;
import frc.robot.commands.Shoot;
import frc.robot.commands.ShootMoving;
import frc.robot.commands.Spin;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Ramasseur;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterLoop;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.Unjammer;

public class RobotContainer {
        // =========================
        // Manette
        // =========================
        private final CommandXboxController joystick = new CommandXboxController(0);

        // =========================
        // Drive constants
        // =========================
        private final double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        private final double maxAngularRate = RotationsPerSecond.of(2).in(RadiansPerSecond);

        private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
                        .withDeadband(maxSpeed * 0.1)
                        .withRotationalDeadband(maxAngularRate * 0.1)
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        private final Telemetry telemetry = new Telemetry(maxSpeed);

        // =========================
        // Subsystems sont déclarés ici
        // =========================
        public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
        public final Ramasseur ramasseur = new Ramasseur();
        public final TurretSubsystem turret = new TurretSubsystem();
        public final Spindexer spindexer = new Spindexer();
        public final Shooter shooter = new Shooter();
        public final Unjammer unjammer = new Unjammer();
        public final ShooterLoop shooterLoop = new ShooterLoop(drivetrain, shooter, turret);

        // =========================
        // Constantes partagées
        // =========================
        public static final Records.ShooterParams SHOOTER_DEFAULTS = Constants.Shooter.defaultParams();

        // =========================
        // Autonome
        // =========================
        private final SendableChooser<Command> autoChooser;

        public RobotContainer() {
                configureDefaultCommands();
                configureBindings();
                configureDashboard();

                autoChooser = AutoBuilder.buildAutoChooser();
                SmartDashboard.putData("Auto Chooser", autoChooser);
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

                joystick.back().onTrue(Commands.runOnce(() -> drivetrain.resetPose(new Pose2d(3, 4, Rotation2d.kZero))));
        }

        private void configureDashboard() {
                drivetrain.registerTelemetry(telemetry::telemeterize);
        }

        // =========================
        // Bindings par subsystem
        // =========================
        private void configureDriveBindings() {
                joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        }

        private void configureTurretBindings() {
                joystick.povUp().onTrue(turret.setTurretPosition(-150));
                joystick.povLeft().onTrue(turret.setTurretPosition(-90));
                joystick.povRight().onTrue(turret.setTurretPosition(90));
                joystick.povDown().onTrue(turret.setTurretPosition(160));
                joystick.y().toggleOnTrue(turret.home());
        }

        private void configureShooterBindings() {
                joystick.a().whileTrue(buildMovingShootCommand());
                joystick.b().onTrue(shooter.homeHood());
        }

        private void configureIntakeBindings() {
                joystick.x().toggleOnTrue(new Ramasser(ramasseur));
        }

        private void configureDisabledBindings() {
                RobotModeTriggers.disabled().onTrue(
                                Commands.runOnce(turret::stopTourelle, turret));
        }

        // =========================
        // Commands (TODO: BOUGER VERS UN FICHIER COMMAND)
        // =========================
        private Command buildMovingShootCommand() {
                return Commands.sequence(
                        new Shoot(shooter).withTimeout(1),
                        Commands.parallel(
                                        new ShootMoving(shooter, turret, shooterLoop),
                                        new Spin(spindexer),
                                        new Ramasser(ramasseur),
                                        Commands.runEnd(
                                                        () -> unjammer.set(0.5),
                                                        () -> unjammer.set(0.0),
                                                        unjammer))
                );
        }

        // =========================
        // Driver input
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
        // Hook public
        // =========================
        public Command getInitCommand() {
                return Commands.sequence(
                                turret.home(),
                                turret.setTurretPosition(0));
        }

        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }
}
