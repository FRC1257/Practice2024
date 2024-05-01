package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import frc.robot.util.Exceptions.VoltageToleranceException;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;

import lombok.Getter;
import lombok.Setter;


public class Shooter extends SubsystemBase {
    private static final LoggedTunableNumber Kp =
      new LoggedTunableNumber("Shooter/Kp", ShooterConstants.RealController.Kp());
    private static final LoggedTunableNumber Ki =
      new LoggedTunableNumber("Shooter/Ki", ShooterConstants.RealController.Ki());
    private static final LoggedTunableNumber Kd =
      new LoggedTunableNumber("Shooter/Kd", ShooterConstants.RealController.Kd());
     private static final LoggedTunableNumber Ks =
      new LoggedTunableNumber("Shooter/Ks", ShooterConstants.RealController.Ks());
     private static final LoggedTunableNumber Kv =
      new LoggedTunableNumber("Shooter/Kv", ShooterConstants.RealController.Kv());
     private static final LoggedTunableNumber Ka =
      new LoggedTunableNumber("Shooter/Ka", ShooterConstants.RealController.Ka());
    private static final LoggedTunableNumber shooterTolerance =
      new LoggedTunableNumber("Shooter/ToleranceRPM", ShooterConstants.SHOOTER_RPM_TOLERANCE);

    private final ShooterIO shooterIO;
    private final ShooterIOInputsAutoLogged shooterInputs = new ShooterIOInputsAutoLogged();

    @Getter @Setter private boolean characterizing = false;
 
    private double leftSetpointRPM, rightSetpointRPM = 0;
    private double leftMotorVoltage, rightMotorVoltage = 0;

    public Shooter(ShooterIO io) {
        shooterIO = io;
        shooterIO.setPID(Kp.get(), Ki.get(), Kd.get());
        shooterIO.setFF(Ks.get(), Kv.get(), Ka.get());
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
            () -> shooterIO.setPID(Kp.get(), Ki.get(), Kd.get()),
            Kp,
            Ki,
            Kd);
        LoggedTunableNumber.ifChanged(
            hashCode(),
            () -> shooterIO.setFF(Ks.get(), Kv.get(), Ka.get()),
            Ks,
            Kv,
            Ka);

        shooterIO.updateInputs(shooterInputs);
        Logger.processInputs("Shooter", shooterInputs);

        if (DriverStation.isDisabled()) {
            shooterIO.stop();
        }
    }

    public void runVolts(double volts) {
        shooterIO.setVoltage(volts);
    }

    public double getLeftCharacterizationVelocity() {
        return shooterInputs.lShooterVelocityRPM;
    }
    
    public double getRightCharacterizationVelocity() {
        return shooterInputs.rShooterVelocityRPM;
    }

    /**
     * @return {@code true} if the shooter reaches it's setpoint: {@code false} otherwise.
     */
    @AutoLogOutput(key = "Shooter/AtSetpoint")
    public boolean atSetpoint() {
        return Math.abs(shooterInputs.lShooterVelocityRPM - leftSetpointRPM)
                <= shooterTolerance.get()
            && Math.abs(shooterInputs.rShooterVelocityRPM - rightSetpointRPM)
                <= shooterTolerance.get();
    }

    public Command runVoltage(DoubleSupplier volts) {
        return new FunctionalCommand(
        () -> setVoltage(volts), 
        () -> {setVoltage(volts);},
        (interrupted) -> {
            if (interrupted) {
            shooterIO.stop();
            }
        },
        () -> false,
        this
        );
    }
    public Command runVoltage(double volts) {
        return runVoltage(() -> volts);
      }
    
    public void setVoltage(DoubleSupplier Voltage) {

        leftMotorVoltage = Voltage.getAsDouble()*10;
        rightMotorVoltage = Voltage.getAsDouble()*10;

        shooterIO.setVoltage(Voltage.getAsDouble()*10);

        isVoltageRightClose(Voltage.getAsDouble());
        isVoltageLeftClose(Voltage.getAsDouble());

    }
    
    public void setVoltage(double volts){
        setVoltage(() -> volts);
    }
    
     
    public void setRPM(double RPM) {
        leftSetpointRPM = RPM; //RPM setpoint is being set here
        rightSetpointRPM = RPM;
        shooterIO.setRPM(RPM);
    }
    
      public void setRPM(double rpm) {
        setRPM(rpm);
      }
    
      public void setRPM(DoubleSupplier RPM) {
        leftSetpointRPM = RPM.getAsDouble();
        rightSetpointRPM = RPM.getAsDouble();
        shooterIO.setRPM(RPM.getAsDouble());
      }
    
      public Command stop() {
        return runVoltage(() -> 0);
      }
    
      public double getLeftSpeedMetersPerSecond() {
        return shooterInputs.lShooterVelocityRPM * ShooterConstants.wheelRadiusM * 2 * Math.PI / 60;
      }
    
      public double getRightSpeedMetersPerSecond() {
        return shooterInputs.rShooterVelocityRPM * ShooterConstants.wheelRadiusM * 2 * Math.PI / 60;
      }
    
      @AutoLogOutput(key = "Shooter/ReadyToShoot")
      public boolean readyToShoot() {
        return shooterInputs.rShooterVelocityRPM > 5_000 && shooterInputs.lShooterVelocityRPM > 5_000;
      }



}
