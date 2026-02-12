package frc.mentor.robot;

/**
 * API exposée aux jeunes (records / structs)
 *
 * - Records.VisionMeas
 * - Records.ShooterInputs
 * - Records.ShooterSetpoints
 * - Records.ShooterStatus
 * - Records.IntakeSetpoints
 * - Records.IntakeStatus
 * - Records.IndexerSetpoints
 * - Records.IndexerStatus
 * - Records.DriveSetpoints
 * - Records.DriveStatus
 * - Records.ValidationResult.success()
 * - Records.ValidationResult.error(code,msg)
 * - Records.ErrorStatus.none()
 * - Records.ErrorStatus.of(code,msg)
 */

/**
 * Exemples de records complexes qu’on pourrais utiliser.
 *
 * 1) ShooterOutput (setpoints + status + infos utiles pour debug)
 *
 * public record ShooterOutput(
 * Records.ShooterSetpoints setpoints, // rpm/hood/yaw demandés
 * Records.ShooterStatus status, // ready/inZone + error/confidence
 * double distanceM, // distance calculée vers la cible
 * double timeOfFlightSec // temps de vol estimé (pour shoot-on-move)
 * ) {}
 *
 * Exemple d’usage (côté jeunes):
 * ShooterOutput out = shooterLogic.update(inputs);
 * shooterSubsystem.apply(out.setpoints());
 * Log.value("Shooter/DistanceM", out.distanceM());
 * Log.value("Shooter/TOF", out.timeOfFlightSec());
 *
 *
 * 2) PoseEstimate (fusion 4 caméras + pose estimator)
 *
 * public record PoseEstimate(
 * edu.wpi.first.math.geometry.Pose2d pose,
 * double timestampSec, // timestamp pose
 * double latencySec, // latence vision
 * Records.Confidence confidence, // 0-1
 * int[] contributingTagIds, // tags utilisés (pour debug)
 * Records.ErrorStatus error
 * ) {}
 *
 * Exemple d’usage:
 * PoseEstimate pe = visionFusion.getPoseEstimate();
 * driveSubsystem.addVision(pe.pose(), pe.timestampSec());
 * Log.value("Vision/Confidence", pe.confidence().value());
 * Log.value("Vision/Tag0", pe.contributingTagIds().length > 0 ?
 * pe.contributingTagIds()[0] : -1);
 */

public final class RecordsREADME {
    private RecordsREADME() {
    }
}