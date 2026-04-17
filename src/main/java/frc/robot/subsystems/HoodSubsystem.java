package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.utils.MathUtils;
import frc.robot.Constants;

public final class HoodSubsystem extends SubsystemBase {
    // =========================
    // Hardware
    // =========================
    private final TalonFX hoodMotor = new TalonFX(Constants.Hood.MOTOR_ID, Constants.CAN.rio);


    // =========================
    // Requests
    // =========================
    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0.0).withSlot(0);
    

    public HoodSubsystem() {
        applyConfigs();
    }
    

    // =========================
    // Setup
    // =========================
    private void applyConfigs() {
        hoodMotor.getConfigurator().apply(Constants.Hood.CONFIG);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Hood/Courent Deg", getAngleDeg());
    }

    // =========================
    // Position control
    // =========================

    /**
     * Convertit l'angle cible de la tourelle en rotations moteur.
     *
     * Le zéro moteur est placé au limit switch, qui correspond à l'angle max mécanique.
     * Donc en s'éloignant du switch, les rotations moteur deviennent négatives.
     */
    public void setTargetAngleDeg(double targetDeg) {
        double clampedDeg = MathUtils.clamp(
                targetDeg,
                Constants.Hood.ANGLE_MIN_DEG,
                Constants.Hood.ANGLE_MAX_DEG);

        double hoodDegFromHome = clampedDeg;
        double hoodRotationsFromHome = hoodDegFromHome / Constants.Hood.DEGREES_PER_REVOLUTION;
        double motorRotations = hoodRotationsFromHome * Constants.Hood.RATIO;

        hoodMotor.setControl(motionMagicRequest.withPosition(motorRotations));
    }

    /**
     * Reconvertit la position moteur actuelle en angle réel de tourelle.
     *
     * Comme le zéro moteur est au home, on repart de l'angle max mécanique.
     */
    public double getAngleDeg() {
        double motorRotations = hoodMotor.getPosition().getValueAsDouble();
        double hoodRotationsFromHome = motorRotations / Constants.Hood.RATIO;
        double hoodDegFromHome = hoodRotationsFromHome * Constants.Hood.DEGREES_PER_REVOLUTION;
         SmartDashboard.putNumber("Hood/hoodDegFromHome", hoodDegFromHome);

        return MathUtils.clamp(
                hoodDegFromHome,
                Constants.Hood.ANGLE_MIN_DEG,
                Constants.Hood.ANGLE_MAX_DEG);
    }

    public double getAngleRad() {
        return Math.toRadians(getAngleDeg());
    }

    public boolean isAtAngleDeg(double targetDeg) {
        return isAtAngleDeg(targetDeg, Constants.Hood.DEFAULT_ANGLE_TOLERANCE_DEG);
    }

    public boolean isAtAngleDeg(double targetDeg, double toleranceDeg) {
        return Math.abs(getAngleDeg() - targetDeg) <= Math.abs(toleranceDeg);
    }

    // =========================
    // Manual control
    // =========================
    public void setManualOutput(double output) {
        hoodMotor.set(output);
    }

    public void stop() {
        hoodMotor.stopMotor();
    }


    public void resetMotorPosition(double positionRotations) {
        hoodMotor.setPosition(positionRotations);
    }

   


    // =========================
    // Convenience commands
    // =========================
    public Command setTargetAngleCommand(double angleDegrees) {
        return Commands.runOnce(() -> setTargetAngleDeg(angleDegrees), this);
    }
}