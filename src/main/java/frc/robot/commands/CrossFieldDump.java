package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.utils.MathUtils;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CommandSwerveDrivetrain.DriveMode;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SpindexerSubsystem;
import frc.robot.subsystems.TurretSubsystem;

/**
 * Commande de dump : tire les balles vers notre mur d'alliance depuis n'importe où sur le terrain.
 *
 * <p>
 * Fonctionnement général :
 * <ul>
 * <li>La turret s'oriente pour que les balles volent TOUT DROIT le long de l'axe X du terrain
 * (vers notre mur d'alliance), en compensant la vitesse de translation du robot.</li>
 * <li>Le RPM de la flywheel est ajusté linéairement selon la distance du mur :
 * {@link Constants.Commands#CROSS_DUMP_RPM_AT_OWN_WALL} près de notre mur,
 * {@link Constants.Commands#CROSS_DUMP_RPM_AT_FAR_WALL} du côté adverse.</li>
 * <li>L'alimentation (feed + indexer) est bloquée si le turret est dans son angle mort
 * mécanique ou si le flywheel n'a pas encore atteint son rpm.</li>
 * </ul>
 */
public final class CrossFieldDump extends Command {

    private final ShooterSubsystem shooter;
    private final CommandSwerveDrivetrain drivetrain;
    private final TurretSubsystem turret;
    private final SpindexerSubsystem spindexer;
    private final HoodSubsystem hood;

    public CrossFieldDump(
            ShooterSubsystem shooter,
            CommandSwerveDrivetrain drivetrain,
            TurretSubsystem turret,
            SpindexerSubsystem spindexer,
            HoodSubsystem hood) {
        this.shooter = shooter;
        this.drivetrain = drivetrain;
        this.turret = turret;
        this.spindexer = spindexer;
        this.hood = hood;

        // On réserve le shooter, le turret et le spindexer — le drivetrain reste libre
        // pour que le pilote puisse continuer à se déplacer pendant le tir.
        addRequirements(shooter, turret, spindexer, hood);
    }

    @Override
    public void initialize() {
        // Sécurité : on s'assure que les mécanismes sont arrêtés avant de démarrer
        // pour éviter de tirer une balle dans le mauvais sens au moment de l'activation.
        spindexer.stopAll();
        shooter.stopShooter();
        shooter.stopKicker();
    }

    @Override
    public void execute() {
        Pose2d pose = drivetrain.getState().Pose;
        boolean isRed = isRedAlliance();

        // ── 1. Direction de tir souhaitée ───────────────────────────────────────────
        // On veut que la balle voyage le long de l'axe X du terrain, droit vers notre mur.
        // Blue : notre mur est à X = 0, donc on tire vers -X (π rad en convention field).
        // Red : notre mur est à X ≈ 16.5 m, donc on tire vers +X (0 rad).
        double desiredYawFieldRad = isRed ? 0.0 : Math.PI;

        // ── 3. Conversion en angle turret (repère mécanique) ─────────────────────────
        // Le turret est référencé par rapport à l'arrière du robot (0° = arrière, CW positif).
        // On soustrait d'abord le yaw du robot pour obtenir un angle relatif au robot, puis
        // on décale de -180° pour passer au repère mécanique du turret.
        double desiredYawRelRad = MathUtils.wrapRad(
                desiredYawFieldRad - pose.getRotation().getRadians());

        double rawTurretTargetDeg = Math.toDegrees(desiredYawRelRad) - 180.0;
        if (rawTurretTargetDeg < -180.0)
            rawTurretTargetDeg += 360.0;

        // On envoie la consigne au turret ; le sous-système la clampera lui-même aux limites.
        turret.setTargetAngleDeg(-rawTurretTargetDeg);

        // ── 5. Calcul du RPM selon la distance au mur ────────────────────────────────
        // Plus on est loin de notre mur, plus la balle doit aller vite pour l'atteindre (ET PAS TROP VITE POUR PAS OVERSHOOT PAR DESSUS LE MUR).
        // On utilise une interpolation linéaire calibrée entre les deux extrémités du terrain.
        double ownWallX = isRed
                ? Constants.Commands.CROSS_DUMP_OWN_WALL_X_RED_M
                : Constants.Commands.CROSS_DUMP_OWN_WALL_X_BLUE_M;
        double distToOwnWallM = isRed
                ? Math.max(0.0, ownWallX - pose.getX())
                : Math.max(0.0, pose.getX() - ownWallX);
                
        SmartDashboard.putNumber("Shooter/distToOwnWallM", distToOwnWallM);

        double rpmCmd = Constants.Commands.CROSS_DUMP_RPM_AT_OWN_WALL + Constants.Commands.CROSS_DUMP_RPM_PER_METER * distToOwnWallM;
        // Sécurité : on clamp pour éviter de sous-charger ou sur-charger le flywheel
        rpmCmd = MathUtils.clamp(rpmCmd,
                Constants.Commands.CROSS_DUMP_RPM_AT_OWN_WALL,
                Constants.Commands.CROSS_DUMP_RPM_AT_FAR_WALL);

        hood.setTargetAngleDeg(Constants.Hood.ANGLE_MAX_DEG);
        shooter.setShooterRpm(rpmCmd);
        shooter.setKickerRpm(Constants.Commands.KICKER_RPM);
        spindexer.setFeedSpeed(Constants.Commands.FEED_SPEED);
        spindexer.setIndexerSpeed(Constants.Commands.SPINDEXER_SPEED);
    }

    @Override
    public void end(boolean interrupted) {
        // Nettoyage : on remet le drivetrain en mode normal et on arrête tous les mécanismes
        // pour ne pas laisser le flywheel tourner inutilement après la commande.
        drivetrain.setDriveMode(DriveMode.NORMAL);
        spindexer.stopAll();
        shooter.stopKicker();
        shooter.stopShooter();
        turret.stop();
    }

    @Override
    public boolean isFinished() {
        // La commande tourne tant que le pilote maintient le bouton.
        // C'est le déclencheur externe (bouton relâché) qui y met fin.
        return false;
    }

    /** Retourne true si le robot joue côté Rouge, false côté Bleu. */
    private static boolean isRedAlliance() {
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
    }
}