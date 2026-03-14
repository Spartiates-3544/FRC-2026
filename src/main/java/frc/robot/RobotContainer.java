package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.subsystems.Ramasseur;
import frc.robot.subsystems.Spindexer;
import frc.lib.robot.Records;
import frc.robot.commands.PositionnerTourelle;
import frc.robot.commands.Ramasser;
import frc.robot.commands.Spin;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.TurretSubsystem;



public class RobotContainer {
    public final TurretSubsystem turret = new TurretSubsystem();
    private final double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private final double MaxAngularRate = RotationsPerSecond.of(2).in(RadiansPerSecond);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1)
            .withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final CommandXboxController joystick = new CommandXboxController(0);
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    
    public final Ramasseur ramasseur = new Ramasseur();
    public static final Records.ShooterParams SHOOTER_DEFAULTS = Constants.Shooter.defaultParams();
    public PositionnerTourelle positionnerTourelle = new PositionnerTourelle(turret);    

    public final Spindexer spindexer = new Spindexer();


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
/*
        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(
                () -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
*/

        joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);

        // joystick.x().onTrue(positionnerTourelle);
        
        joystick.a().toggleOnTrue(new Spin(spindexer));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
