// frc/mentor/robot/Records.java
package frc.mentor.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * Petits "records" partagés entre les fichiers mentors/robot.
 *
 * Pourquoi on les met ici?
 * - Pour éviter les doublons (mêmes records dans plusieurs classes).
 * - Pour que n'importe quel subsystem/command puisse les utiliser facilement.
 * - Pour garder le code du solver/logic plus clean (juste la logique).
 *
 * Note:
 * - Un record en Java est immutable (tu ne modifies pas ses champs).
 * - Si tu veux "changer" quelque chose, tu crées un nouveau record.
 */
public final class Records {
  private Records() {}

  /**
   * Paramètres du shooter + solver.
   *
   * Ça regroupe:
   * - la physique (gravité, air, balle)
   * - la géométrie (position du muzzle, hauteur de release)
   * - les limites mécaniques (min/max turret, hood, rpm)
   * - la logique de tir (latence, tolérance de hit)
   * - les paramètres du solveur "instant" (fenêtres, itérations, etc.)
   *
   * En pratique:
   * - Tu gardes un objet ShooterParams "global" (Constants ou config).
   * - Tu ajustes certains champs live (AdvantageScope / NetworkTables) si tu veux.
   */
  public static record ShooterParams(
      double g,
      double rhoAir,
      double ballMass,
      double ballDiam,

      double Cd,
      boolean enableDrag,

      double releaseHeight,
      double muzzleForwardOffset,
      double muzzleSideOffset,

      double wheelRadiusM,
      double slipFactor,
      double exitSpeedFactor,

      double flywheelRpmMin,
      double flywheelRpmMax,

      double hoodFixedDeg,
      double hoodMinDeg,
      double hoodMaxDeg,

      double turretMinDeg,
      double turretMaxDeg,

      double fireLatencyS,
      double hitRadiusM,

      double rtDt,
      double rtTmax,

      String goalType,
      double goalOpenRadiusM,
      boolean goalRequireDescend,
      boolean goalRejectSideEntry,

      double turretSlewRateDps,
      double hoodRateDps,
      double flywheelAccelRpmS,

      boolean solveForHood,
      boolean solveForRpm,

      double continuityWeight,

      double hoodPreferWeight,
      boolean hoodPreferHigh,
      double hoodTiebreakEps,

      int instantIters,

      int instantYawSamples,
      int instantYawGoldenIters,
      int instantRpmSamples,
      int instantRpmGoldenIters,
      int instantHoodSamples,
      int instantHoodGoldenIters,

      double instantYawWinDegMin,
      double instantYawWinDegMax,

      double instantRpmSpanMin,
      double instantRpmSpanMax,

      double instantHoodSpanMin,
      double instantHoodSpanMax,

      boolean instantWidenIfMiss,
      int instantWidenPasses,
      double instantWidenYawMul,
      double instantWidenRpmMul,

      int hoodPushupSteps,
      double hoodPushupEpsM) {}

  /**
   * État du robot "utile pour tirer".
   *
   * Contient:
   * - position XY sur le field
   * - yaw (orientation)
   * - vitesse XY
   * - vitesse de rotation (omega)
   * - accélération XY (optionnel, mais utile si tu predicts)
   *
   * Le solver va souvent prédire cet état dans le futur (latence + temps de ready).
   */
  public static record RobotState(
      Translation2d posXY,
      Rotation2d yaw,
      Translation2d velXY,
      double omegaRadS,
      Translation2d accelXY) {}

  /**
   * Résultat d'une simulation rapide de trajectoire.
   *
   * - hit: est-ce que ça "rentre" selon ton modèle de goal
   * - t: temps où le miss est minimum (ou temps de hit)
   * - missM: distance minimale à la cible (mètres)
   * - sideEntry: cas spécial "entrée par le côté" (si tu veux le rejeter)
   */
  public static record SimResult(
      boolean hit,
      double t,
      double missM,
      boolean sideEntry) {}

  /**
   * État actuel des actuateurs du shooter.
   *
   * C'est ce que tu lis sur le robot:
   * - angle turret (relatif au robot)
   * - angle hood (degrés)
   * - flywheel rpm
   *
   * Le solver utilise ça pour estimer "combien de temps avant d'être prêt".
   */
  public static record ActuatorState(
      double turretYawRelRad,
      double hoodDeg,
      double flywheelRpm) {}

  /**
   * Solution finale "utilisable par les students".
   *
   * ok:
   * - true si on considère que le tir est valide (hit ou miss <= tolérance)
   *
   * turretYawRelRad / hoodDeg / flywheelRpm:
   * - setpoints à envoyer aux moteurs
   *
   * tFireS:
   * - "quand" tirer (délai total: latence + ready)
   *
   * tofS:
   * - time-of-flight (approx) du projectile (si ton modèle le renvoie)
   *
   * missM:
   * - combien on manque, en mètres (0 = parfait)
   *
   * info:
   * - petit texte pour debug (ex: "hit", "instant", "pushup", etc.)
   */
  public static record ShotSolution(
      boolean ok,
      double turretYawRelRad,
      double flywheelRpm,
      double hoodDeg,
      double tFireS,
      double tofS,
      double missM,
      String info) {}

  /**
   * Debug "mentor only" pour comprendre ce que le solver a fait.
   *
   * Exemple d'usage:
   * - logger ça dans AdvantageScope pour tuner.
   *
   * tReadyS:
   * - temps estimé pour que turret/hood/rpm atteignent la solution
   *
   * robotPred:
   * - état du robot prédit à tFireS
   *
   * seedYawRad:
   * - yaw initial ("seed") avant raffinement
   *
   * distXY:
   * - distance horizontale muzzle -> cible
   */
  public static record Debug(
      double tReadyS,
      double tFireS,
      RobotState robotPred,
      double seedYawRad,
      double distXY,
      boolean hit,
      double missM,
      double exitSpeed) {}

  /**
   * Sortie complète du solveur.
   *
   * - solution: ce que tu utilises pour commander le shooter
   * - debug: extra pour tuner et analyser
   */
  public static record SolveOutput(ShotSolution solution, Debug debug) {}

  /**
   * Modèle minimal de la cible.
   *
   * Ici c'est juste un point 3D (x,y,z) en coord field.
   * Tu peux étendre plus tard si tu veux:
   * - type de goal
   * - rayon d'ouverture
   * - orientation, etc.
   */
  public static record TargetModel(Translation3d targetXYZ) {}
}
