package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.lib.robot.Records;
import frc.lib.robot.Records.ActuatorState;

public class Shooter extends SubsystemBase{ 

    private TalonFX kickerMotor = new TalonFX(5);
    private TalonFX shooterMotor1 = new TalonFX(6);
    private TalonFX shooterMotor2 = new TalonFX(7);
    private TalonFX hoodMoteur = new TalonFX(9);
// private DigitalInput switchHaut = new DigitalInput(2); seulement une switch en bas 
    private DigitalInput switchBas = new DigitalInput(8);

    private final VoltageOut sysIdRequest = new VoltageOut(0.0);
    private final SysIdRoutine sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,        // Use default ramp rate (1 V/s)
                Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
                null,        // Use default timeout (10 s)
                            // Log state with Phoenix SignalLogger class
                (state) -> SignalLogger.writeString("state", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                (volts) -> kickerMotor.setControl(sysIdRequest.withOutput(volts.in(Volts))),
                null,
                this
            )
        );


    public Shooter() {
        setName("Shooter");

        kickerMotor.getConfigurator().apply(Constants.Shooter.kickerConfigs);
        shooterMotor1.getConfigurator().apply(Constants.Shooter.shooterConfigs);

        shooterMotor2.getConfigurator().apply(Constants.Shooter.shooterConfigs);
        Follower followRequest = new Follower(6, MotorAlignmentValue.Aligned);
        shooterMotor2.setControl(followRequest);

        hoodMoteur.getConfigurator().apply(Constants.Shooter.hoodConfigs);
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }

    public void setShooterSpeed(double shooterSpeedRPM) {
       VelocityVoltage demande = new VelocityVoltage(shooterSpeedRPM/60).withSlot(0);
       shooterMotor1.setControl(demande);
    }

    public void setKickerSpeed(double kickerSpeedRPM) {
        VelocityVoltage demande = new VelocityVoltage(kickerSpeedRPM/60).withSlot(0);
        kickerMotor.setControl(demande);
    }

    // hoodAngle: degrees 
    public void setHoodAngle(double hoodAngle) { 
        Records.ShooterParams p = Constants.Shooter.defaultParams();
        if (hoodAngle > p.hoodMinDeg() && hoodAngle < p.hoodMaxDeg()) {
            double toursHood = hoodAngle / 360.0;
            double toursMoteur = toursHood * Constants.Shooter.hoodRatio;
            PositionVoltage demande = new PositionVoltage(toursMoteur)
                                            .withSlot(0)
                                            .withLimitReverseMotion(switchBas.get());
            hoodMoteur.setControl(demande);
        } // todo: add else
        
    }    

    /** Move elevator at given speed (-1.0 to 1.0) */
    public void setHoodMoteur(double speed) {
        hoodMoteur.set(speed);
    }

    /** Stop the elevator motor */
    public void stopHood() {
        hoodMoteur.stopMotor();
    }

    /** Returns true if the limit switch is pressed */
    public boolean isHoodAtHome() {
        return switchBas.get(); 
    }

    /** Reset encoder position to zero */
    public void resetHoodPosition() {
        hoodMoteur.setPosition(0);
    }

    /** Simulated encoder update (replace with real encoder code) */
    public void updatePosition(double delta) {
//        position += delta;
    }

    public double getHoodPosition() {
        return hoodMoteur.getPosition().getValueAsDouble();
    }

    public Command setShooterMoteur(double shooterSpeed){
        return Commands.runOnce(() -> setShooterSpeed(shooterSpeed), this);
    }

    public void setKicker(double speed) {
        kickerMotor.set(speed);
    }

    public void setShooter(double speed) {
        shooterMotor1.set(speed);
    }

    public Command homeHood() {
        return Commands.run(() -> hoodMoteur.set(0.10), this).until(() -> isHoodAtHome()).finallyDo(() -> hoodMoteur.stopMotor());
    }

    public double getShooterRPM() {
        return shooterMotor1.getVelocity().getValueAsDouble() * 60.0;
    }
}

   
