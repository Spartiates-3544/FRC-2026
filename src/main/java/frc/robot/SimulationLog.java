package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

/**
 * Centralise les logs de simulation pour le turret, le hood et le shooter.
 * Publié sur SmartDashboard/Glass sous la clé "SimLog/".
 *
 * <p>La flèche "turretAim" apparaît dans le widget Field2d (table "Pose") au même
 * endroit que le robot, orientée dans la direction où pointe le turret en
 * coordonnées terrain. Sélectionner "turretAim" dans le panneau Objects de Glass.
 *
 * <p>Convention d'angle mécanique du turret : 0° = arrière du robot.
 * Angle terrain du turret = cap robot + 180° + angle turret.
 * Si la flèche pointe dans le mauvais sens (miroir), inverser le signe de
 * {@code turretDeg} dans le calcul de {@code turretFieldDeg}.
 */
public class SimulationLog extends SubsystemBase {

    private final CommandSwerveDrivetrain drivetrain;
    private final TurretSubsystem turret;
    private final HoodSubsystem hood;
    private final ShooterSubsystem shooter;

    // =========================
    // Field2d turret arrow (same "Pose" table as Telemetry)
    // =========================
    private final DoubleArrayPublisher turretAimPub = NetworkTableInstance.getDefault()
            .getTable("Pose")
            .getDoubleArrayTopic("turretAim")
            .publish();
    // Required so Glass recognises the table as a Field2d (Telemetry already sets this,
    // but we publish it here too in case SimulationLog is constructed first).
    @SuppressWarnings("unused")
    private final StringPublisher fieldTypePub = NetworkTableInstance.getDefault()
            .getTable("Pose")
            .getStringTopic(".type")
            .publish();

    private final double[] turretAimArray = new double[3];

    public SimulationLog(
            CommandSwerveDrivetrain drivetrain,
            TurretSubsystem turret,
            HoodSubsystem hood,
            ShooterSubsystem shooter) {
        this.drivetrain = drivetrain;
        this.turret = turret;
        this.hood = hood;
        this.shooter = shooter;

        fieldTypePub.set("Field2d");
    }

    @Override
    public void periodic() {
        double turretDeg = turret.getLastCommandedAngleDeg();
        double hoodDeg = hood.getLastCommandedAngleDeg();
        double shooterRpm = shooter.getLastCommandedShooterRpm();

        SmartDashboard.putNumber("SimLog/Turret Angle Deg", turretDeg);
        SmartDashboard.putNumber("SimLog/Hood Angle Deg", hoodDeg);
        SmartDashboard.putNumber("SimLog/Shooter Commanded RPM", shooterRpm);

        // Publish turret aim pose on the Field2d.
        // Turret 0° = back of robot → add 180° to convert to robot-forward frame,
        // then add robot heading to get field frame.
        Pose2d robotPose = drivetrain.getState().Pose;
        double turretFieldDeg = robotPose.getRotation().getDegrees() + 180.0 + turretDeg;
        Pose2d turretAimPose = new Pose2d(robotPose.getTranslation(), Rotation2d.fromDegrees(turretFieldDeg));

        turretAimArray[0] = turretAimPose.getX();
        turretAimArray[1] = turretAimPose.getY();
        turretAimArray[2] = turretAimPose.getRotation().getDegrees();
        turretAimPub.set(turretAimArray);
    }
}
