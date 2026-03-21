package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.fasterxml.jackson.databind.jsonFormatVisitors.JsonObjectFormatVisitor;
import com.pathplanner.lib.auto.AutoBuilder;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.Ramasser;
import frc.robot.commands.Shoot;
import frc.robot.subsystems.Ramasseur;
import frc.lib.robot.Records;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spindexer;
import frc.robot.commands.PositionnerTourelle;
import frc.robot.commands.Spin;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.TurretSubsystem;

public class RobotContainer {
    private final CommandXboxController joystick = new CommandXboxController(0);

    private final double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private final double MaxAngularRate = RotationsPerSecond.of(2).in(RadiansPerSecond);
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.1)
        .withRotationalDeadband(MaxAngularRate * 0.1)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    
    public final Ramasseur ramasseur = new Ramasseur();
    public static final Records.ShooterParams SHOOTER_DEFAULTS = Constants.Shooter.defaultParams();
    public final TurretSubsystem turret = new TurretSubsystem();

    public final Spindexer spindexer = new Spindexer();
    public final Shooter shooter = new Shooter();

    private SendableChooser<Command> autoChooser;

    public RobotContainer() {
        configureBindings();
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        //  NamedCommands.registerCommand("ramasser", new Ramasser(ramasseur));

    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() -> drive
                        .withVelocityX(-Math.copySign(Math.pow(joystick.getLeftY(),2), joystick.getLeftY()) * MaxSpeed)
                        .withVelocityY(-Math.copySign(Math.pow(joystick.getLeftX(),2), joystick.getLeftX()) * MaxSpeed)
                        .withRotationalRate(-(joystick.getRightTriggerAxis()-joystick.getLeftTriggerAxis()) * MaxAngularRate)));

        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        joystick.x().toggleOnTrue(new Ramasser(ramasseur));
      
        // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.b().whileTrue(drivetrain.applyRequest(
        //         () -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        drivetrain.registerTelemetry(logger::telemeterize);

        joystick.povUp().onTrue(turret.setTurretPosition(-150));
        joystick.povLeft().onTrue(turret.setTurretPosition(-90));
        joystick.povRight().onTrue(turret.setTurretPosition(90));
        joystick.povDown().onTrue(turret.setTurretPosition(160));
        joystick.y().toggleOnTrue(turret.home());

        RobotModeTriggers.disabled().onTrue(Commands.run(() -> turret.stopTourelle(), turret));
        joystick.b().toggleOnTrue(Commands.run(() -> {
                shooter.setKicker(1);
                shooter.setShooter(1);
        }, shooter).finallyDo(() -> {
                shooter.setKicker(0);
                shooter.setShooter(0);
        }));
      
    //    joystick.a().toggleOnTrue(new Spin(spindexer));

        joystick.a().toggleOnTrue(new Shoot(shooter));
        
        // joystick.leftBumper().onTrue(Commands.runOnce(() -> SignalLogger.start()));
        // joystick.rightBumper().onTrue(Commands.runOnce(() -> SignalLogger.stop()));
        // joystick.povUp().whileTrue(shooter.sysIdQuasistatic(Direction.kForward));
        // joystick.povDown().whileTrue(shooter.sysIdQuasistatic(Direction.kReverse));
        // joystick.povLeft().whileTrue(shooter.sysIdDynamic(Direction.kForward));
        // joystick.povRight().whileTrue(shooter.sysIdDynamic(Direction.kReverse));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
