package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;

import frc.lib.logging.ExtendedLogger;
import frc.lib.robot.Tunables;
import frc.lib.utils.MathUtils;

public final class TurretSubsystem extends SubsystemBase {
    private static TalonFX turret = new TalonFX(8);

    @Tunables.TunableNum(key = "Shooter/Turret-kP", def = 60.0, hz = 2, clamp = false)
    private double turretkP = 60;

    @Tunables.TunableNum(key = "Shooter/Turret-kD", def = 2.0, hz = 2, clamp = false)
    private double TurretkD = 2.0;

    @Tunables.TunableNum(key = "Shooter/TurretRatio", def = 13.0, hz = 2, clamp = false)
    private double ratio = 13.0;

    @Tunables.TunableNum(key = "Shooter/TurretMinDeg", def = -160.0, hz = 2, clamp = false)
    private double minDeg = 13.0;

    @Tunables.TunableNum(key = "Shooter/TurretMaxDeg", def = 160.0, hz = 2, clamp = false)
    private double maxDeg = 13.0;

    @ExtendedLogger.LoggableField(path = "Shooter/CurrentPos")
    private double targetDegLog = 0.0;

     @ExtendedLogger.LoggableField(path = "Shooter/CurrentPos")
    private double posDegLog = 0.0;

     @ExtendedLogger.LoggableField(path = "Shooter/CurrentPos")
    private double posMotorRotLog = 0.0;

     @ExtendedLogger.LoggableField(path = "Shooter/CurrentPos")
    private double cmdMotorRotLog = 0.0;

    private final MotionMagicVoltage turretMM = new MotionMagicVoltage(0);

    private double turretTargetDeg = 0.0;
    private static double ratioX = 13;

    public TurretSubsystem() {
        ExtendedLogger.registerInstance(this);
        Tunables.registerInstance(this);
        applyConfigs();
    }

    private void applyConfigs() {
        var turretCFG = new TalonFXConfiguration();
        turretCFG.MotorOutput = new MotorOutputConfigs().withNeutralMode(com.ctre.phoenix6.signals.NeutralModeValue.Brake);
        turretCFG.Slot0 = new Slot0Configs()
                .withKP(turretkP)
                .withKD(TurretkD);

        turretCFG.MotionMagic = new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(2)
                .withMotionMagicAcceleration(8);

        turret.getConfigurator().apply(turretCFG);
    }

    @Override
    public void periodic() {
        CurrentPosLog = turret.getPosition().getValueAsDouble();
        double ClampedTurretTargetDeg = MathUtils.clamp(turretTargetDeg, -160.0, 160.0);
        turret.setControl(turretMM.withPosition(turretDegPerMotorTurn(ClampedTurretTargetDeg)));

    }

    private static double turretDegPerMotorTurn(double deg) {
        double mechRot = deg / 360;
        return mechRot * ratioX;
    }

    public void setTurretDeg(double deg) {
        turretTargetDeg = deg;
    }

}
