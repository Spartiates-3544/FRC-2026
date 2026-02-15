// frc/mentor/robot/Records.java
package frc.mentor.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * <p>
 * Records "data only" utilisés par la logique mentor (shooter, solver, sim).
 * </p>
 *
 * Idée:
 * - Ici on met juste des structures de données (records), pas de logique.
 * - Ça facilite: tests, simulation, logging, debug.
 */
public final class Records {
        private Records() {
        }

        /**
         * <p>
         * Paramètres globaux du shooter + solver + simulation de trajectoire.
         * </p>
         *
         * Notes générales:
         * - Tout est en unités SI (m, s, kg) sauf les angles hood/turret en degrés et
         * flywheel en RPM.
         * - Ces valeurs sont pensées pour être "tunable": tu changes ici et tout le
         * solver suit.
         *
         * @param g
         *                accélération gravitationnelle (m/s^2). Exemple: 9.80665.
         * @param rhoAir
         *                densité de l'air (kg/m^3). Exemple: 1.225.
         * @param ballMass
         *                masse du projectile (kg). Exemple: 0.2267.
         * @param ballDiam
         *                diamètre du projectile (m). Exemple: 0.127 (5").
         *
         * @param Cd
         *                coefficient de traînée (drag) (sans unité). Plus haut = plus de drag.
         * @param enableDrag
         *                active la simulation avec traînée de l'air. false = balistique "idéal" (vacuum) sauf gravité.
         *
         * @param releaseHeight
         *                hauteur de sortie de la balle (m) par rapport au sol.
         * @param muzzleForwardOffset
         *                offset du muzzle vers l'avant du robot (m), dans le frame du robot.
         * @param muzzleSideOffset
         *                offset du muzzle vers la gauche/droite du robot (m), dans le frame du robot.
         *
         * @param wheelRadiusM
         *                rayon de la roue du flywheel (m). Sert à convertir RPM en vitesse linéaire.
         * @param slipFactor
         *                facteur de "slip" (sans unité). &lt; 1.0 si la balle sort plus lente que la surface de roue.
         * @param exitSpeedFactor
         *                facteur multiplicatif final sur la vitesse de sortie (sans unité). Pratique pour calibrer.
         *
         * @param flywheelRpmMin
         *                RPM minimum permis.
         * @param flywheelRpmMax
         *                RPM maximum permis.
         *
         * @param hoodFixedDeg
         *                angle de hood fixe (deg) si tu ne solves pas le hood (solveForHood=false).
         * @param hoodMinDeg
         *                angle minimum permis du hood (deg).
         * @param hoodMaxDeg
         *                angle maximum permis du hood (deg).
         *
         * @param turretMinDeg
         *                yaw minimum permis de la turret relatif au robot (deg).
         * @param turretMaxDeg
         *                yaw maximum permis de la turret relatif au robot (deg).
         *
         * @param fireLatencyS
         *                délai "commande quand balle quitte le shooter" (s).
         *                Exemple: temps de latence du trigger + délai interne + délais mécaniques.
         *                Le solver utilise ça pour prédire où le robot va être au moment du tir.
         * @param hitRadiusM
         *                rayon d'acceptation (m) pour dire c'est un hit (tolérance).
         *
         * @param rtDt
         *                pas de temps de la sim rapide (s). Plus petit = plus précis mais plus lourd CPU.
         * @param rtTmax
         *                temps max simulé (s) avant d'abandonner la trajectoire.
         *
         * @param goalType
         *                type de modèle de goal. Exemple: "top_entry" (entrée par le haut) ou autre.
         *                Le code ShooterLogic.simulateMissFast change son test de hit selon ça.
         * @param goalOpenRadiusM
         *                rayon d'ouverture du goal (m) (utile surtout pour "top_entry").
         * @param goalRequireDescend
         *                si true, pour compter un hit il faut que la balle soit en train de descendre (vz &lt; 0) au crossing.
         * @param goalRejectSideEntry
         *                si true, on rejette une entrée "par le côté" (ex: entrer sous le plan sans passer par le haut).
         *
         * @param turretSlewRateDps
         *                vitesse max de la turret (deg/s). Sert à estimer combien de temps pour être "ready" et pour simuler la dynamique actuateur.
         * @param hoodRateDps
         *                vitesse max du hood (deg/s).
         * @param flywheelAccelRpmS
         *                accélération max du flywheel (RPM/s).
         *
         * @param solveForHood
         *                si true, le solver cherche le meilleur hood (sinon hoodFixedDeg).
         * @param solveForRpm
         *                si true, le solver cherche le meilleur RPM sinon il garde un guess/valeur fixe selon ton usage).
         *
         * @param continuityWeight
         *                poids de la continuité (stabilité) dans la fonction de coût.
         *                Plus haut = le solver préfère rester proche de la solution précédente (moins de "jitter"), mais peut accepter un peu plus de miss.
         *
         * @param hoodPreferWeight
         *                poids d'une préférence de hood (sans unité). Sert à "pousser" le solver vers hood haut/bas quand plusieurs solutions se valent.
         * @param hoodPreferHigh
         *                si true, on préfère un hood plus haut; si false on préfère plus bas.
         * @param hoodTiebreakEps
         *                epsilon (m) utilisé pour départager deux coûts presque égaux: on applique la préférence hood.
         *
         * @param instantIters
         *                nombre de passes internes pour raffiner yaw/rpm/hood.
         *                Plus haut = plus de CPU, mais souvent plus robuste.
         *
         * @param instantYawSamples
         *                nombre d'échantillons pour bracketer un minimum en yaw (plus = plus stable, plus lourd).
         * @param instantYawGoldenIters
         *                nombre d'itérations de golden search en yaw (plus = plus précis).
         * @param instantRpmSamples
         *                nombre d'échantillons pour bracketer en RPM.
         * @param instantRpmGoldenIters
         *                nombre d'itérations golden search RPM.
         * @param instantHoodSamples
         *                nombre d'échantillons pour bracketer en hood.
         * @param instantHoodGoldenIters
         *                nombre d'itérations golden search hood.
         *
         * @param instantYawWinDegMin
         *                fenêtre yaw min (deg) autour du seed (petit = plus stable, peut manquer si seed mauvais).
         * @param instantYawWinDegMax
         *                fenêtre yaw max (deg).
         *
         * @param instantRpmSpanMin
         *                span min (RPM) autour du RPM guess pour la recherche.
         * @param instantRpmSpanMax
         *                span max (RPM).
         *
         * @param instantHoodSpanMin
         *                span min (deg) autour du hood guess.
         * @param instantHoodSpanMax
         *                span max (deg).
         *
         * @param instantWidenIfMiss
         *                si true, si on miss on élargit la fenêtre et on réessaie (plus robuste, plus lourd CPU).
         * @param instantWidenPasses
         *                nombre de "widen passes" max.
         * @param instantWidenYawMul
         *                multiplicateur de fenêtre yaw à chaque widen.
         * @param instantWidenRpmMul
         *                multiplicateur de span RPM à chaque widen.
         *
         * @param hoodPushupSteps
         *                nombre d'étapes pour la logique "pushup" hood (si tu l'utilises ailleurs).
         *                Généralement: quand la solution est borderline, tu peux "pousser" hood un peu.
         * @param hoodPushupEpsM
         *                epsilon en mètres utilisé par le pushup (tolérance sur miss).
         *
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
                        double hoodPushupEpsM) {
        }

        /**
         * <p>
         * État du robot (pour le solver).
         * </p>
         *
         * @param posXY
         *                position XY sur le terrain (m).
         * @param yaw
         *                orientation du robot sur le terrain (Rotation2d).
         * @param velXY
         *                vitesse XY (m/s).
         * @param omegaRadS
         *                vitesse angulaire du robot (rad/s).
         * @param accelXY
         *                accélération XY (m/s^2). Peut être (0,0) si tu ne l'utilises
         *                pas.
         */
        public static record RobotState(
                        Translation2d posXY,
                        Rotation2d yaw,
                        Translation2d velXY,
                        double omegaRadS,
                        Translation2d accelXY) {
        }

        /**
         * <p>
         * Résultat d'une simulation rapide de trajectoire (simulateMissFast).
         * </p>
         *
         * @param hit
         *                true si la trajectoire passe dans la tolérance de hit.
         * @param t
         *                temps (s) où le meilleur point a eu lieu (hit time si hit).
         * @param missM
         *                distance d'erreur minimale (m) trouvée pendant la
         *                trajectoire.
         * @param sideEntry
         *                true si on a détecté une entrée "par le côté" (si activé).
         * @param missXY_m
         *                composante XY de l'erreur (m) au meilleur point.
         * @param missZ_m
         *                composante Z de l'erreur (m) au meilleur point.
         */
        public static record SimResult(
                        boolean hit,
                        double t,
                        double missM,
                        boolean sideEntry,
                        double missXY_m,
                        double missZ_m) {
        }

        /**
         * <p>
         * État des actuateurs du shooter (pour estimer readiness + dynamique).
         * </p>
         *
         * @param turretYawRelRad
         *                yaw actuel de la turret relatif au robot (rad).
         * @param hoodDeg
         *                angle actuel du hood (deg).
         * @param flywheelRpm
         *                RPM actuel du flywheel.
         */
        public static record ActuatorState(
                        double turretYawRelRad,
                        double hoodDeg,
                        double flywheelRpm) {
        }

        /**
         * <p>
         * Solution finale du solver: ce qu'on veut commander au robot.
         * </p>
         *
         * @param ok
         *                true si la solution est considérée valide (hit ou miss
         *                dans tolérance).
         *                Sur le robot, tu peux aussi le forcer false si blind
         *                hold actif, etc.
         * @param turretYawRelRad
         *                yaw désiré relatif robot (rad).
         * @param flywheelRpm
         *                RPM désiré.
         * @param hoodDeg
         *                hood désiré (deg).
         * @param tFireS
         *                temps total estimé avant le tir (s) = fireLatency +
         *                temps pour être ready.
         * @param tofS
         *                temps de vol (s) de la balle (approx/mesuré par la
         *                sim).
         * @param missM
         *                miss estimé (m).
         * @param info
         *                texte debug simple ("hit", "instant", "blind_hold",
         *                etc.).
         */
        public static record ShotSolution(
                        boolean ok,
                        double turretYawRelRad,
                        double flywheelRpm,
                        double hoodDeg,
                        double tFireS,
                        double tofS,
                        double missM,
                        String info) {
        }

        /**
         * <p>
         * Debug complet (optionnel) renvoyé par le solver.
         * </p>
         *
         * Sert à:
         * - comprendre pourquoi ça choisit tel yaw/rpm/hood
         * - grapher dans AdvantageScope
         * - tuner les paramètres (continuity, fenêtres, spans, etc.)
         *
         * @param tReadyS
         *                temps estimé (s) pour que turret/hood/flywheel
         *                atteignent les setpoints.
         * @param tFireS
         *                temps total estimé (s) avant de tirer (latence + ready).
         * @param robotPred
         *                état du robot prédit au moment du tir.
         *
         * @param seedYawRad
         *                yaw seed initial (rad) avant optimisation.
         * @param distXY
         *                distance XY (m) du muzzle au target (utile pour tuning
         *                des fenêtres).
         *
         * @param hit
         *                true si la sim indique un hit.
         * @param missM
         *                miss minimal (m).
         * @param exitSpeed
         *                vitesse de sortie estimée (m/s) correspondant au RPM
         *                choisi.
         *
         * @param turretClamped
         *                true si yaw brut a été clampé par les limites turret.
         * @param rpmClamped
         *                true si RPM brut a été clampé par min/max ou slew per
         *                tick.
         * @param hoodClamped
         *                true si hood brut a été clampé par min/max.
         *
         * @param yawCmdDeg
         *                yaw commandé (deg) après clamp.
         * @param yawRawDeg
         *                yaw brut (deg) avant clamp.
         * @param rpmCmd
         *                RPM commandé après clamp/slew.
         * @param rpmRaw
         *                RPM brut avant clamp/slew.
         * @param hoodCmdDeg
         *                hood commandé (deg) après clamp.
         * @param hoodRawDeg
         *                hood brut (deg) avant clamp.
         *
         * @param missXY_m
         *                composante XY de l'erreur (m).
         * @param missZ_m
         *                composante Z de l'erreur (m).
         *
         * @param sideEntry
         *                true si entrée par le côté a été détectée (top entry).
         * @param goalType
         *                type de goal utilisé (string, ex: "top_entry").
         * @param info
         *                info debug courte.
         */
        public static record Debug(
                        double tReadyS,
                        double tFireS,
                        RobotState robotPred,

                        double seedYawRad,
                        double distXY,

                        boolean hit,
                        double missM,
                        double exitSpeed,

                        boolean turretClamped,
                        boolean rpmClamped,
                        boolean hoodClamped,

                        double yawCmdDeg,
                        double yawRawDeg,
                        double rpmCmd,
                        double rpmRaw,
                        double hoodCmdDeg,
                        double hoodRawDeg,

                        double missXY_m,
                        double missZ_m,

                        boolean sideEntry,
                        String goalType,
                        String info) {
        }

        /**
         * <p>
         * Output complet d'un solve: solution + debug.
         * </p>
         *
         * @param solution
         *                solution finale (setpoints).
         * @param debug
         *                debug détaillé (peut être null selon implémentation).
         */
        public static record SolveOutput(ShotSolution solution, Debug debug) {
        }

        /**
         * <p>
         * Modèle de target (super simple pour l'instant).
         * </p>
         *
         * @param targetXYZ
         *                position de la cible sur le terrain (m).
         */
        public static record TargetModel(Translation3d targetXYZ) {
        }
}
