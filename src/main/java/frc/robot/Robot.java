package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.robot.LedStrips;

public class Robot extends TimedRobot {

    private Command m_autonomousCommand;
    private final RobotContainer m_robotContainer;

    public Robot() {
        m_robotContainer = new RobotContainer();
        LedStrips.init(0, 60);
    }

    @Override
    public void robotInit() {
    }

    @Override
    public void robotPeriodic() {
        
        double robotPeriodicStart = Timer.getFPGATimestamp();

        CommandScheduler.getInstance().run();

        double robotPeriodicEnd = Timer.getFPGATimestamp();
        double robotPeriodicMs = (robotPeriodicEnd - robotPeriodicStart) * 1000.0;
        SmartDashboard.putNumber("Timing/RobotPeriodicMs", robotPeriodicMs);
    }

    @Override
    public void disabledInit() {
        // Clear the cached goal so it is recomputed when the next alliance report
        // arrives. Important during practice sessions where alliance may change.
        FieldTargets.clearCache();
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
        CommandScheduler.getInstance().schedule(m_robotContainer.getInitCommand());
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