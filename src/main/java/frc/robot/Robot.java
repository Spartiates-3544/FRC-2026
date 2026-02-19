// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.logging.ExtendedLogger;
import frc.lib.robot.Tunables;

/**
 * Robot.java
 *
 * "Main loop" du robot (WPILib).
 *
 * À retenir :
 * - robotPeriodic() roule à ~50Hz (toutes les 20ms)
 * - CommandScheduler = exécute les Commands et appelle periodic() des Subsystems
 * - Tunables.run() = met à jour les valeurs live depuis NetworkTables
 * - ExtendedLogger.run() = log automatiquement les @LoggableField
 */
public class Robot extends TimedRobot {

    /** Command autonome courante (peut être null). */
    private Command m_autonomousCommand;

    /** Contient tous les subsystems + commandes + bindings manette. */
    private final RobotContainer m_robotContainer;

    /**
     * Outil CTRE pour logger/rejouer le timing + inputs manette.
     * (Utile pour debug sans robot réel)
     */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
            .withTimestampReplay()
            .withJoystickReplay();

    public Robot() {
        // On construit RobotContainer 1 seule fois au boot.
        // C’est là que les subsystems sont créés.
        m_robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        // 1) Mise à jour CTRE replay/log (si utilisé)
        m_timeAndJoystickReplay.update();

        // 2) Lit les tunables (/Tuning/...) et applique les valeurs aux fields annotés
        Tunables.run();

        // 3) Log automatiquement les champs annotés @LoggableField
        ExtendedLogger.run();

        // 4) Fait rouler le framework Command-based:
        // - appelle periodic() des subsystems
        // - exécute les commandes scheduled
        // - gère les triggers / buttons
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
        // Appelé 1 fois quand on passe en Disabled
    }

    @Override
    public void disabledPeriodic() {
        // Appelé à chaque loop pendant Disabled
    }

    @Override
    public void disabledExit() {
        // Appelé 1 fois quand on sort de Disabled
    }

    @Override
    public void autonomousInit() {
        // On demande au RobotContainer "quelle commande autonome on veut"
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        // Si on a une commande, on la schedule
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {
        // Généralement vide: le scheduler fait la job
    }

    @Override
    public void autonomousExit() {
        // Appelé 1 fois quand on sort d’Autonomous
    }

    @Override
    public void teleopInit() {
        // Important: on annule l'auto quand on arrive en Teleop
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
    }

    @Override
    public void teleopPeriodic() {
        // Généralement vide: le scheduler fait la job
    }

    @Override
    public void teleopExit() {
        // Appelé 1 fois quand on sort de Teleop
    }

    @Override
    public void testInit() {
        // Mode test: souvent on cancel tout pour partir clean
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
        // Boucle test
    }

    @Override
    public void testExit() {
        // Appelé 1 fois quand on sort de Test
    }

    @Override
    public void simulationPeriodic() {
        // Simulation seulement (pas sur le robot)
    }
}
