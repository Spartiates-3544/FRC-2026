package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

import frc.robot.commands.RunIntake;
import frc.robot.commands.SetTurretAngle;
import frc.robot.commands.DeployIntake;
import frc.robot.commands.ShootNow;
import frc.robot.commands.ShootMoving;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CommandSwerveDrivetrain.DriveMode;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SpindexerSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.UnjammerSubsystem;
import frc.robot.subsystems.control.ShooterLoop;
import frc.robot.commands.DumpIntoAllianceZone;

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

        private final SwerveRequest.FieldCentricFacingAngle driveFaceTranslation = new SwerveRequest.FieldCentricFacingAngle()
                        .withDeadband(maxSpeed * 0.1)
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        private Rotation2d faceTranslationHeading = Rotation2d.kZero;
        private DriveMode lastDriveMode = DriveMode.NORMAL;
        private double lastDriveCommandBuildMs = 0.0;

        private final Telemetry telemetry = new Telemetry(maxSpeed);

        private boolean shouldSlowForShootNow() {
                return CommandScheduler.getInstance().isScheduled(shootNowCommand)
                                || CommandScheduler.getInstance().isScheduled(shootMovingCommand);
        }

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
        public final LedSubsystem leds = new LedSubsystem(turret);

        // =========================
        // Autonomous
        // =========================
        private final SendableChooser<Command> autoChooser;

        private void configureNamedCommands() {
                // Ramasser: ouvre et roule l'intake + l'indexeur pendant max 3s
                NamedCommands.registerCommand("run-intake",
                                (new RunIntake(intake, leds)));

                NamedCommands.registerCommand("deploy-intake",
                                (new DeployIntake(intake)));

                NamedCommands.registerCommand("prepare-for-shot",
                                new ShootMoving(shooter, turret, shooterLoop, leds));

                NamedCommands.registerCommand("shoot-now",
                                new ShootNow(intake, spindexer, unjammer, shooterLoop, leds));
        }

        public RobotContainer() {
                shootNowCommand = new ShootNow(intake, spindexer, unjammer, shooterLoop);
                shootMovingCommand = new ShootMoving(shooter, turret, shooterLoop);
                configureDefaultCommands();
                configureBindings();
                configureDashboard();
                configureNamedCommands();
                autoChooser = AutoBuilder.buildAutoChooser();
                SmartDashboard.putData("Auto Chooser", autoChooser);
                intake.open();

                // Shooter YAW + RPM automatique lorsque dans la zone
                // shooter.setDefaultCommand(buildMovingShootCommand());
        }

        // =========================
        // Setup
        // =========================
        private void configureDefaultCommands() {
                driveFaceTranslation.HeadingController.setPID(8.0, 0.0, 0.2);
                driveFaceTranslation.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

                drivetrain.setDefaultCommand(
                                drivetrain.applyRequest(() -> {
                                        double startTime = Timer.getFPGATimestamp();

                                        double vx = getDriveX();
                                        double vy = getDriveY();
                                        double omega = getDriveOmega();

                                        if (shouldSlowForShootNow()) {
                                                vx *= Constants.Commands.SHOOT_NOW_TRANSLATION_SCALE;
                                                vy *= Constants.Commands.SHOOT_NOW_TRANSLATION_SCALE;
                                                omega *= Constants.Commands.SHOOT_NOW_ROTATION_SCALE;
                                        }

                                        DriveMode mode = drivetrain.getDriveMode();

                                        if (mode != lastDriveMode) {
                                                if (mode == DriveMode.FACE_TRANSLATION) {
                                                        faceTranslationHeading = drivetrain.getState().Pose
                                                                        .getRotation();
                                                }
                                                lastDriveMode = mode;
                                        }

                                        SwerveRequest request;
                                        if (mode == DriveMode.FACE_TRANSLATION) {
                                                double translationMag = Math.hypot(vx, vy);

                                                if (translationMag > 0.05 * maxSpeed) {
                                                        faceTranslationHeading = Rotation2d
                                                                        .fromRadians(Math.atan2(vy, vx));
                                                }

                                                request = driveFaceTranslation
                                                                .withVelocityX(vx)
                                                                .withVelocityY(vy)
                                                                .withTargetDirection(faceTranslationHeading);
                                        } else {
                                                request = driveRequest
                                                                .withVelocityX(vx)
                                                                .withVelocityY(vy)
                                                                .withRotationalRate(omega);
                                        }

                                        double endTime = Timer.getFPGATimestamp();
                                        lastDriveCommandBuildMs = (endTime - startTime) * 1000.0;
                                        SmartDashboard.putNumber("Timing/DriveCommandBuildMs", lastDriveCommandBuildMs);

                                        return request;
                                }));

                final var idle = new SwerveRequest.Idle();
                RobotModeTriggers.disabled().whileTrue(
                                drivetrain.applyRequest(() -> idle).ignoringDisable(true));
        }

        private void configureBindings() {
                configureDriveBindings();
                configureShooterBindings();
                configureIntakeBindings();
                configureDisabledBindings();
        }

        private void configureDashboard() {
                drivetrain.registerTelemetry(telemetry::telemeterize);
        }

        // =========================
        // Drive bindings
        // =========================
        private void configureDriveBindings() {
                joystick.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        }

        // =========================
        // Shooter bindings
        // =========================
        private final Command shootNowCommand;
        private final Command shootMovingCommand;

        private void configureShooterBindings() {
                joystick.a().whileTrue(shootNowCommand);
                joystick.y().toggleOnTrue(shootMovingCommand);
                joystick.b().whileTrue(
                                new DumpIntoAllianceZone(
                                                shooter,
                                                drivetrain,
                                                turret,
                                                spindexer,
                                                intake,
                                                unjammer,
                                                leds));
        }

        private Command buildMovingShootCommand() {
                return new ShootMoving(shooter, turret, shooterLoop, leds);
        }

        // =========================
        // Intake bindings
        // =========================
        private void configureIntakeBindings() {
                joystick.x().onTrue(new RunIntake(intake));
        }

        // =========================
        // Disabled bindings
        // =========================
        private void configureDisabledBindings() {
                RobotModeTriggers.disabled().onTrue(
                                Commands.runOnce(() -> {
                                        drivetrain.setDriveMode(DriveMode.NORMAL);
                                        turret.stop();
                                        shooter.stopShooter();
                                        shooter.stopKicker();
                                        shooter.stopHood();
                                        intake.stop();
                                        spindexer.stopAll();
                                        unjammer.stop();
                                }, drivetrain, turret, shooter, intake, spindexer, unjammer));
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
                                turret.setTargetAngleCommand(0.0))
                                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
        }

        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }

        @SuppressWarnings("unused")
        private Pose2d allianceFlipPose(Pose2d pose) {
                double FIELD_LENGTH_IN = 651.22;
                double FIELD_LENGTH_M = Units.inchesToMeters(FIELD_LENGTH_IN);
                return new Pose2d(
                                FIELD_LENGTH_M - pose.getX(),
                                pose.getY(),
                                pose.getRotation().plus(Rotation2d.fromDegrees(180.0)));
        }
}