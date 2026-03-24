package frc.lib.archives;

import edu.wpi.first.math.geometry.Translation3d;
//import frc.lib.logging.ExtendedLogger;
import frc.lib.robot.Records;
import frc.robot.Constants;

/**
 * ShooterAim:
 * Thin stateless-ish aiming wrapper around BallisticSolver.
 *
 * Behavior:
 * - Loop.update() is intended to run every 20 ms (50 Hz)
 * - No internal throttling
 * - Reads live ShooterParams every call
 * - Keeps a warm-start solution for continuity
 */
public final class ShooterAim {
    private ShooterAim() {
    }

    public static record AimResult(
            Records.ShotSolution desired,
            Records.ShotSolution cmd,
            boolean ok,
            boolean blindHold,
            Records.ShotSolution nextWarm,
            Records.Debug debug) {

        public double yawDesiredDeg() {
            return Math.toDegrees(desired.turretYawRelRad());
        }

        public double yawCmdDeg() {
            return Math.toDegrees(cmd.turretYawRelRad());
        }

        public double rpmCmd() {
            return cmd.flywheelRpm();
        }

        public double hoodCmdDeg() {
            return cmd.hoodDeg();
        }

        public double missM() {
            return cmd.missM();
        }
    }

    /**
     * No-throttle loop:
     * - pulls ShooterTuning params every update
     * - solves every update
     * - preserves warm start
     */
    public static final class Loop {

        //@ExtendedLogger.LoggableField(path = "ShooterAim/last", hz = 2)
        //private volatile AimResult lastLogged = null;

        //@ExtendedLogger.LoggableField(path = "ShooterAim/holdYawRelDeg", hz = 2)
       // private volatile double holdYawRelDegLogged = 0.0;

        private final Records.ShooterParams params;
        private Records.ShotSolution warm = null;
        private AimResult last = null;

        public Loop() {
            this(Constants.Shooter.PARAMS);
        }

        public Loop(Records.ShooterParams params) {
            this.params = params;
           // ExtendedLogger.registerInstance(this);
        }

        public AimResult last() {
            return last;
        }

        public void resetWarm() {
            warm = null;
        }

        public AimResult update(
                Records.RobotState robotNow,
                Records.ActuatorState actNow,
                Translation3d targetXYZ,
                double holdYawRelRad) {

            last = ShooterAim.solve(
                    params,
                    robotNow,
                    actNow,
                    targetXYZ,
                    warm,
                    holdYawRelRad);

            warm = last.nextWarm();

           // lastLogged = last;
           // holdYawRelDegLogged = Math.toDegrees(holdYawRelRad);

            return last;
        }
    }

    /**
     * Stateless solve helper.
     */
    public static AimResult solve(
            Records.ShooterParams p,
            Records.RobotState robotNow,
            Records.ActuatorState actNow,
            Translation3d targetXYZ,
            Records.ShotSolution warm,
            double holdYawRelRad) {

        Records.SolveOutput out = BallisticSolver.solve(p, robotNow, actNow, targetXYZ, warm);
        Records.ShotSolution desired = out.solution();

        boolean blindHold = ShooterLogic.isTurretBlindSpot(p, desired.turretYawRelRad());

        double yawCmd = ShooterLogic.applyBlindHoldYaw(
                p,
                desired.turretYawRelRad(),
                holdYawRelRad);

        boolean okFinal = desired.ok() && !blindHold;

        Records.ShotSolution cmd = new Records.ShotSolution(
                okFinal,
                yawCmd,
                desired.flywheelRpm(),
                desired.hoodDeg(),
                desired.tFireS(),
                desired.tofS(),
                desired.missM(),
                blindHold ? "blind_hold" : desired.info());

        Records.ShotSolution nextWarm = desired;

        return new AimResult(desired, cmd, okFinal, blindHold, nextWarm, out.debug());
    }
}