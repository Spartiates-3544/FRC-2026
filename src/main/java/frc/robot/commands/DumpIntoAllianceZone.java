package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.utils.MathUtils;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CommandSwerveDrivetrain.DriveMode;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SpindexerSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.UnjammerSubsystem;

public final class DumpIntoAllianceZone extends Command {
    private final ShooterSubsystem shooter;
    private final CommandSwerveDrivetrain drivetrain;
    private final TurretSubsystem turret;
    private final SpindexerSubsystem spindexer;
    private final IntakeSubsystem intake;
    private final UnjammerSubsystem unjammer;

    // Position approximative du mur alliance sur le terrain (m)
    private static final double BLUE_ALLIANCE_WALL_X_M = 0.25;
    private static final double RED_ALLIANCE_WALL_X_M = 16.25;

    // Tolérances
    private static final double TURRET_TOL_DEG = 5.0;
    private static final double FLYWHEEL_TOL_RPM = 250.0;

    public DumpIntoAllianceZone(
            ShooterSubsystem shooter,
            CommandSwerveDrivetrain drivetrain,
            TurretSubsystem turret,
            SpindexerSubsystem spindexer,
            IntakeSubsystem intake,
            UnjammerSubsystem unjammer) {
        this.shooter = shooter;
        this.drivetrain = drivetrain;
        this.turret = turret;
        this.spindexer = spindexer;
        this.intake = intake;
        this.unjammer = unjammer;

        addRequirements(shooter, turret, spindexer, intake, unjammer);
    }

    @Override
    public void initialize() {
        drivetrain.setDriveMode(DriveMode.FACE_TRANSLATION);

        intake.open();
        intake.stop();
        spindexer.stopAll();
        unjammer.stop();
        shooter.stopShooter();
        shooter.stopKicker();
    }

    @Override
    public void execute() {
        Pose2d pose = drivetrain.getState().Pose;
        boolean isRed = isRedAlliance();

        // On pointe vers notre propre mur alliance
        double desiredYawFieldRad = isRed ? Math.PI : 0.0;
        double desiredYawRelRad = MathUtils.wrapRad(
                desiredYawFieldRad - pose.getRotation().getRadians());

        double turretTargetDeg = Math.toDegrees(desiredYawRelRad) - 180.0;
        if (turretTargetDeg < -180.0) turretTargetDeg += 360.0;
        turret.setTargetAngleDeg(turretTargetDeg);

        // RPM selon distance au mur alliance
        double wallX = isRed ? RED_ALLIANCE_WALL_X_M : BLUE_ALLIANCE_WALL_X_M;
        double distanceToZoneM = isRed
                ? Math.max(0.0, wallX - pose.getX())
                : Math.max(0.0, pose.getX() - wallX);

        double rpmCmd = Constants.Commands.DUMP_RPM_AT_WALL + Constants.Commands.DUMP_RPM_PER_METER * distanceToZoneM;
        rpmCmd = MathUtils.clamp(rpmCmd, Constants.Commands.DUMP_RPM_MIN, Constants.Commands.DUMP_RPM_MAX);

        shooter.setShooterRpm(rpmCmd);
        shooter.setKickerRpm(Constants.Commands.KICKER_RPM);

        double turretErrDeg = Math.abs(turret.getAngleDeg() - turretTargetDeg);
        double flywheelErrRpm = Math.abs(shooter.getShooterRpm() - rpmCmd);

        boolean ready = turretErrDeg <= TURRET_TOL_DEG
                && flywheelErrRpm <= FLYWHEEL_TOL_RPM;

        // Intake/unjammer roulent tout le temps
        intake.setSpeed(Constants.Commands.INTAKE_SPEED);
        unjammer.setSpeed(Constants.Commands.UNJAMMER_SPEED);

        // Feed et indexe seulement quand prêt
        if (ready) {
            spindexer.setFeedSpeed(Constants.Commands.FEED_SPEED);
            spindexer.setIndexerSpeed(Constants.Commands.SPINDEXER_SPEED);
        } else {
            spindexer.setFeedSpeed(0.0);
            spindexer.setIndexerSpeed(0.0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setDriveMode(DriveMode.NORMAL);
        intake.stop();
        intake.close();
        spindexer.stopAll();
        unjammer.stop();
        shooter.stopKicker();
        shooter.stopShooter();
        turret.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    private static boolean isRedAlliance() {
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
    }
}