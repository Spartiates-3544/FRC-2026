package frc.robot;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;

public class MatchTelemetry implements Sendable {

	@Override
	public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Match");
		builder.addDoubleProperty("Match Time", Timer::getMatchTime, null);
	}
    
}
