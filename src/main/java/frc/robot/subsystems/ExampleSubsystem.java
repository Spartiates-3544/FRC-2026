// frc/robot/subsystems/ClimberSubsystem.java
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.mentor.logging.ExtendedLogger;
import frc.mentor.robot.Tunables;
import frc.mentor.utils.MathUtils;

/**
 * ClimberSubsystem (EXEMPLE ULTRA SIMPLE)
 *
 * Objectif pédagogique:
 * - Un climber = 1 moteur + 2 limit switches (haut/bas)
 * - Une petite machine d'états (IDLE / MANUAL / HOMING)
 * - Tunables pour modifier des paramètres live (/Tuning/Climber/...)
 * - Logging clair pour debug dans AdvantageScope
 *
 */
public final class ExampleSubsystem extends SubsystemBase {

    // =========================================================
    // Tunables (modifiable en live dans les Network Tables)
    // =========================================================

    /** Voltage max qu'on autorise pour le climber (sécurité). */
    @Tunables.TunableNum(key = "Climber/maxVolts", def = 8.0, hz = 2, clamp = true, min = 0.0, max = 12.0)
    private double maxVolts = 8.0;

    /** Vitesse manuelle “UP” en volts (avant clamp). */
    @Tunables.TunableNum(key = "Climber/manualUpVolts", def = 6.0, hz = 2, clamp = true, min = 0.0, max = 12.0)
    private double manualUpVolts = 6.0;

    /** Vitesse manuelle “DOWN” en volts (avant clamp). */
    @Tunables.TunableNum(key = "Climber/manualDownVolts", def = 4.0, hz = 2, clamp = true, min = 0.0, max = 12.0)
    private double manualDownVolts = 4.0;

    /** Vitesse de “homing” vers le bas (pour trouver le switch bas). */
    @Tunables.TunableNum(key = "Climber/homeDownVolts", def = 2.0, hz = 2, clamp = true, min = 0.0, max = 6.0)
    private double homeDownVolts = 2.0;

    /** Petit délai après avoir touché le switch bas (évite rebond). */
    @Tunables.TunableNum(key = "Climber/homeSettleSec", def = 0.15, hz = 1, clamp = true, min = 0.0, max = 1.0)
    private double homeSettleSec = 0.15;

    // =========================================================
    // Logging (état runtime)
    // =========================================================

    @ExtendedLogger.LoggableField(path = "Climber/state")
    private String stateLogged = "IDLE";

    @ExtendedLogger.LoggableField(path = "Climber/cmdVolts")
    private double cmdVoltsLogged = 0.0;

    @ExtendedLogger.LoggableField(path = "Climber/limitTop")
    private boolean limitTopLogged = false;

    @ExtendedLogger.LoggableField(path = "Climber/limitBottom")
    private boolean limitBottomLogged = false;

    @ExtendedLogger.LoggableField(path = "Climber/isHomed")
    private boolean isHomedLogged = false;

    @ExtendedLogger.LoggableField(path = "Climber/manualRequest")
    private double manualRequestLogged = 0.0; // -1..+1

    // =========================================================
    // Machine d’états (logique simple)
    // =========================================================

    private enum State {
        IDLE, MANUAL, HOMING
    }

    private State state = State.IDLE;

    // Demande manuelle venant d’une commande: -1 = down, +1 = up
    private double manualRequest = 0.0;

    // “homed” = on a trouvé le bas au moins une fois
    private boolean isHomed = false;

    private double homeHitTimeSec = -1.0;

    // =========================================================
    // Constructeur
    // =========================================================

    public ExampleSubsystem() {
        ExtendedLogger.registerInstance(this);
        Tunables.registerInstance(this);
    }

    // =========================================================
    // API simple (utilisée par les Commands)
    // =========================================================

    /** Arrêt immédiat. */
    public void stop() {
        manualRequest = 0.0;
        state = State.IDLE;
    }

    /**
     * Mode manuel: tu passes une valeur -1..+1.
     * Exemple joystick:
     * - trigger droit = +1 (monter)
     * - trigger gauche = -1 (descendre)
     */
    public void setManual(double requestMinus1To1) {
        manualRequest = MathUtils.clamp(requestMinus1To1, -1.0, +1.0);
        state = State.MANUAL;
    }

    /** Lance une séquence “homing” simple: descend jusqu’au limit bas. */
    public void startHoming() {
        homeHitTimeSec = -1.0;
        state = State.HOMING;
    }

    public boolean isHomed() {
        return isHomed;
    }

    // =========================================================
    // Boucle principale
    // =========================================================

    @Override
    public void periodic() {
        // 1) Lire capteurs (ici: stubs)
        boolean limitTop = readLimitTop();
        boolean limitBottom = readLimitBottom();

        // 2) Log sensors
        limitTopLogged = limitTop;
        limitBottomLogged = limitBottom;

        // 3) Choisir commande (volts)
        double cmdVolts = 0.0;

        switch (state) {
            case IDLE -> {
                cmdVolts = 0.0;
            }

            case MANUAL -> {
                manualRequestLogged = manualRequest;

                // Convertit demande -1..+1 en volts (deux tunables séparés)
                if (manualRequest > 0.05) {
                    cmdVolts = manualUpVolts * manualRequest; // monter
                } else if (manualRequest < -0.05) {
                    cmdVolts = -manualDownVolts * (-manualRequest); // descendre (négatif)
                } else {
                    cmdVolts = 0.0;
                }

                // Sécurité simple avec limit switches
                // - si on est en haut et qu’on veut monter -> bloque
                if (limitTop && cmdVolts > 0.0)
                    cmdVolts = 0.0;

                // - si on est en bas et qu’on veut descendre -> bloque
                if (limitBottom && cmdVolts < 0.0)
                    cmdVolts = 0.0;
            }

            case HOMING -> {
                // Idée du homing:
                // - On descend doucement jusqu’à toucher le switch bas
                // - Une fois touché, on attend homeSettleSec
                // - Puis on passe IDLE et on mark isHomed=true

                if (!limitBottom) {
                    cmdVolts = -homeDownVolts; // descend
                    homeHitTimeSec = -1.0; // pas encore touché
                } else {
                    // Touché!
                    if (homeHitTimeSec < 0.0) {
                        homeHitTimeSec = Timer.getFPGATimestamp();
                    }

                    // On garde 0V pendant le “settle”
                    cmdVolts = 0.0;

                    double elapsed = Timer.getFPGATimestamp() - homeHitTimeSec;
                    if (elapsed >= homeSettleSec) {
                        isHomed = true;
                        state = State.IDLE;
                    }
                }
            }
        }

        // 4) Clamp global (sécurité) + application
        cmdVolts = MathUtils.clamp(cmdVolts, -maxVolts, +maxVolts);
        setMotorVoltage(cmdVolts);

        // 5) Logs finaux
        cmdVoltsLogged = cmdVolts;
        isHomedLogged = isHomed;
        stateLogged = state.name();
    }

    private boolean readLimitTop() {
        // Remplace par:
        // return limitTopSwitch.get();
        return false;
    }

    private boolean readLimitBottom() {
        // Remplace par:
        // return limitBottomSwitch.get();
        return false;
    }

    private void setMotorVoltage(double volts) {
        // Remplace par:
        // climberMotor.setVoltage(volts);
    }
}