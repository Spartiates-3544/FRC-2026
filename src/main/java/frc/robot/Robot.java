package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.logging.ExtendedLogger;
import frc.lib.robot.LedStrips;

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

    private Command m_autonomousCommand;
    private final RobotContainer m_robotContainer;

    public Robot() {
        m_robotContainer = new RobotContainer();

        LedStrips.init(0, 60);
    }

    @Override
    public void robotInit() {
        // Background logging thread (off main)
        ExtendedLogger.startBackground(20.0);
    }

    @Override
    public void robotPeriodic() {
        // m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() { 
    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
    }

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
    }

    @Override
    public void simulationPeriodic() {
    }
}