package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.robot.LedStrips;
import frc.lib.robot.LedStrips.LedColor;

/**
 * Manages status LED priority and output.
 *
 * Commands call one of the request*() methods each execute() cycle.
 * periodic() picks the highest-priority pending request and applies it.
 * The turret deadzone check always overrides everything else.
 *
 * Priority (lower number = higher priority):
 *   0  — DEADZONE      red flash
 *   10 — SHOOT_NOW     green flash
 *   20 — DUMP          blue / blue flash when locked
 *   30 — INTAKE        purple
 *   40 — SHOOT_MOVING  yellow (seeking) / green (at target)
 *   99 — IDLE          off
 */
public class LedSubsystem extends SubsystemBase {

    private enum LedState {
        IDLE,
        SHOOT_MOVING_SEEKING,
        SHOOT_MOVING_READY,
        INTAKE,
        DUMP_SEEKING,
        DUMP_READY,
        SHOOT_NOW,
    }

    private static final int PRIORITY_IDLE         = 99;
    private static final int PRIORITY_SHOOT_MOVING = 40;
    private static final int PRIORITY_INTAKE       = 30;
    private static final int PRIORITY_DUMP         = 20;
    private static final int PRIORITY_SHOOT_NOW    = 10;

    private final TurretSubsystem turret;

    // Pending request written by commands in execute(), applied in next periodic()
    private LedState pendingState    = LedState.IDLE;
    private int      pendingPriority = PRIORITY_IDLE;

    public LedSubsystem(TurretSubsystem turret) {
        this.turret = turret;
    }

    // =========================
    // Request API (called from commands)
    // =========================

    public void requestShootNow() {
        accept(LedState.SHOOT_NOW, PRIORITY_SHOOT_NOW);
    }

    public void requestShootMoving(boolean atTarget) {
        accept(atTarget ? LedState.SHOOT_MOVING_READY : LedState.SHOOT_MOVING_SEEKING,
                PRIORITY_SHOOT_MOVING);
    }

    public void requestIntake() {
        accept(LedState.INTAKE, PRIORITY_INTAKE);
    }

    public void requestDump(boolean locked) {
        accept(locked ? LedState.DUMP_READY : LedState.DUMP_SEEKING, PRIORITY_DUMP);
    }

    private void accept(LedState state, int priority) {
        if (priority < pendingPriority) {
            pendingPriority = priority;
            pendingState    = state;
        }
    }

    // =========================
    // Periodic — applies pending request, then resets
    // =========================

    @Override
    public void periodic() {
        if (turret.isInDeadzone()) {
            LedStrips.setLED(LedColor.RED_FLASH);
        } else {
            switch (pendingState) {
                case SHOOT_NOW:
                    LedStrips.setLED(LedColor.GREEN_FLASH);
                    break;
                case SHOOT_MOVING_READY:
                    LedStrips.setLED(LedColor.GREEN);
                    break;
                case SHOOT_MOVING_SEEKING:
                    LedStrips.setLED(LedColor.YELLOW);
                    break;
                case DUMP_READY:
                    LedStrips.setLED(LedColor.BLUE_FLASH);
                    break;
                case DUMP_SEEKING:
                    LedStrips.setLED(LedColor.BLUE);
                    break;
                case INTAKE:
                    LedStrips.setLED(LedColor.PURPLE);
                    break;
                default:
                    LedStrips.setLED(LedColor.ERROR);
                    break;
            }
        }

        // Reset for next frame
        pendingState    = LedState.IDLE;
        pendingPriority = PRIORITY_IDLE;
    }
}
