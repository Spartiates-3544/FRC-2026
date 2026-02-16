// frc/robot/subsystems/ClimberSubsystem.java
package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.mentor.logging.ExtendedLogger;
import frc.mentor.robot.Tunables;
import frc.mentor.utils.MathUtils;

/**
 * ClimberSubsystem (EXEMPLE ULTRA SIMPLE)
 *
 * Objectif pédagogique:
 * - Un climber = 1 moteur + 2 limit switches (haut/bas)
 * - Tunables pour modifier des paramètres live (/Tuning/Climber/...)
 * - Logging clair pour debug dans AdvantageScope
 *
 */
public final class ExampleSubsystem extends SubsystemBase {

    // =========================================================
    // Tunables (modifiable en live dans les Network Tables)
    // =========================================================
    
    /** Vitesse de “homing” vers le bas (pour trouver le switch bas). */
    @Tunables.TunableNum(key = "Climber/homeDownSpeed", def = -0.2, hz = 2, clamp = true, min = -0.5, max = 0.5)
    private double homingSpeed = -0.2;
    
    /** Petit délai après avoir touché le switch bas (évite rebond). */
    @Tunables.TunableNum(key = "Climber/homeSettleSec", def = 0.15, hz = 1, clamp = true, min = 0.0)
    private double homingWaitTimeOnceHomed = 0.15;

    // =========================================================
    // Logging (état runtime)
    // =========================================================

    @ExtendedLogger.LoggableField(path = "Climber/cmdVolts")
    private double cmdVoltsLogged = 0.0;

    @ExtendedLogger.LoggableField(path = "Climber/limitTop")
    private boolean limitTopLogged = false;

    @ExtendedLogger.LoggableField(path = "Climber/limitBottom")
    private boolean limitBottomLogged = false;

    // =========================================================
    // Appareils
    // =========================================================
    private final TalonFX climberMotor = new TalonFX(67);
    private final DigitalInput lowerSwitch = new DigitalInput(0);
    private final DigitalInput upperSwitch = new DigitalInput(1);


    // =========================================================
    // Constructeur
    // =========================================================

    public ExampleSubsystem() {
        ExtendedLogger.registerInstance(this);
        Tunables.registerInstance(this);
    }

    
    // =========================================================
    // Boucle principale
    // =========================================================
    
    @Override
    public void periodic() {
        limitTopLogged = readLimitTop();
        limitBottomLogged = readLimitBottom();
        cmdVoltsLogged = getMotorVoltage();
    }
    
    // =========================================================
    // API simple (utilisée par les Commands)
    // =========================================================

    /**
     * @return une commande déplaçant le grinpeur à sa position de repos (procédure de "homing").
     */
    public Command home() {
        return Commands.run(() -> setMotorSpeed(homingSpeed), this).until(() -> readLimitBottom()).andThen(Commands.waitSeconds(homingWaitTimeOnceHomed));
    }

    /**
     * @return La valeur booléenne (vraie/fausse) de l'interrupteur en haut du ramasseur.
     */
    private boolean readLimitTop() {
        return upperSwitch.get();
    }
    
    /**
     * @return La valeur booléenne (vraie/fausse) de l'interrupteur en bas du ramasseur.
     */
    private boolean readLimitBottom() {
        return lowerSwitch.get();
    }

    /**
     * Définir la valeur de la vitesse du moteur.
     * @param speed la vitesse du moteur (entre -1 et 1).
     */
    private void setMotorSpeed(double speed) {
        speed = MathUtils.clamp(speed, -1, 1);
        climberMotor.set(speed);
    }

    /**
     * Récupérer le voltage aux bornes du moteur (en Volts).
     * @return le voltage aux bornes du moteur (V)
     */
    private double getMotorVoltage() {
        return climberMotor.getMotorVoltage().getValueAsDouble();
    }
}