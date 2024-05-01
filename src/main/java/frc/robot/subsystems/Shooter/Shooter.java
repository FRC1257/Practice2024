package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.util.LoggedTunableNumber;
import lombok.Getter;
import lombok.Setter;


public class Shooter extends SubsystemBase {
    private static final LoggedTunableNumber leftkP =
      new LoggedTunableNumber("Shooter/leftkP", ShooterConstants.RealControllerLeft.Kp());
    private static final LoggedTunableNumber leftkI =
      new LoggedTunableNumber("Shooter/leftkI", ShooterConstants.RealControllerLeft.Ki());
    private static final LoggedTunableNumber leftkD =
      new LoggedTunableNumber("Shooter/leftkD", ShooterConstants.RealControllerLeft.Kd());
     private static final LoggedTunableNumber leftkS =
      new LoggedTunableNumber("Shooter/leftkS", ShooterConstants.RealControllerLeft.Ks());
     private static final LoggedTunableNumber leftkV =
      new LoggedTunableNumber("Shooter/leftkV", ShooterConstants.RealControllerLeft.Kp());
     private static final LoggedTunableNumber leftkA =
      new LoggedTunableNumber("Shooter/leftkA", ShooterConstants.RealControllerLeft.Kp());
    private static final LoggedTunableNumber rightkP =
      new LoggedTunableNumber("Shooter/rightkP",ShooterConstants.RealControllerRight.Kp());
    private static final LoggedTunableNumber rightkI =
      new LoggedTunableNumber("Shooter/rightkI", ShooterConstants.RealControllerRight.Kp());
    private static final LoggedTunableNumber rightkD =
      new LoggedTunableNumber("Shooter/rightkD", ShooterConstants.RealControllerRight.Kp());
    private static final LoggedTunableNumber rightkS =
      new LoggedTunableNumber("Shooter/rightkS", ShooterConstants.RealControllerRight.Kp());
    private static final LoggedTunableNumber rightkV =
      new LoggedTunableNumber("Shooter/rightkV", ShooterConstants.RealControllerRight.Kp());
    private static final LoggedTunableNumber rightkA =
      new LoggedTunableNumber("Shooter/rightkA", ShooterConstants.RealControllerRight.Kp());
    private static final LoggedTunableNumber shooterTolerance =
      new LoggedTunableNumber("Shooter/ToleranceRPM", ShooterConstants.SHOOTER_RPM_TOLERANCE);

    private final ShooterIO shooterIO;
    private final ShooterIOInputsAutoLogged shooterInputs = new ShooterIOInputsAutoLogged();

    @Getter @Setter private boolean characterizing = false;
 
    private double leftSetpointRPM, rightSetpointRPM = 0;
    private double leftMotorVoltage, rightMotorVoltage = 0;

    public Shooter(ShooterIO io) {
        shooterIO = io;
        shooterIO.setLeftPID(leftkP.get(), leftkI.get(), leftkD.get());
        shooterIO.setLeftFF(leftkS.get(), leftkV.get(), leftkA.get());
        shooterIO.setRightFF(rightkS.get(), rightkV.get(), rightkA.get());
        shooterIO.setRightPID(rightkP.get(), rightkI.get(), rightkD.get());
    }

    /**
     * @param Voltage to be compared with the measured voltage
     * @return {@code true} if measured voltage is close to logged voltage. {@code false} otherwise
     */

    @AutoLogOutput(key = "Shooter/CloseRight")
    public boolean isVoltageRightClose(double Voltage) {
        double voltageDifference = Math.abs(Voltage - shooterInputs.rShooterAppliedVolts);
        return voltageDifference <= ShooterConstants.SHOOTER_TOLERANCE;
    }

    /**
     * @param Voltage to be compared with the measured voltage
     * @return {@code true} if measured voltage is close to logged voltage. {@code false} otherwise
     */

    @AutoLogOutput(key = "Shooter/CloseLeft")
    public boolean isVoltageLeftClose(double Voltage) {
        double voltageDifference = Math.abs(Voltage - shooterInputs.lShooterAppliedVolts);
        return voltageDifference <= ShooterConstants.SHOOTER_TOLERANCE;
    }

    @Override
    public void periodic() {
        LoggedTunableNumber.ifChanged(
            hashCode(),
            () -> shooterIO.setLeftPID(leftkP.get(), leftkI.get(), leftkD.get()),
            leftkP,
            leftkI,
            leftkD);
        LoggedTunableNumber.ifChanged(
            hashCode(),
            () -> shooterIO.setLeftFF(leftkS.get(), leftkV.get(), leftkA.get()),
            leftkS,
            leftkV,
            leftkA);
        LoggedTunableNumber.ifChanged(
            hashCode(),
            () -> shooterIO.setRightPID(rightkP.get(), rightkI.get(), rightkD.get()),
            rightkP,
            rightkI,
            rightkD);
        LoggedTunableNumber.ifChanged(
            hashCode(),
            () -> shooterIO.setRightFF(rightkS.get(), rightkV.get(), rightkA.get()),
            rightkS,
            rightkV,
            rightkA);

        shooterIO.updateInputs(shooterInputs);
        Logger.processInputs("Shooter", shooterInputs);

        if (DriverStation.isDisabled()) {
            shooterIO.stop();
        }
    }



}
