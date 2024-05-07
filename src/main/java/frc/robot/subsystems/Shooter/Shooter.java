package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

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

    /**
     * @return left flywheel velocity expressed in revolutions per minute 
     */

    public double getLeftCharacterizationVelocity() {
        return shooterInputs.lShooterVelocityRPM;
    }

    /**
     * @return right flywheel velocity expressed in revolutions per minute 
     */

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

    public Command runVoltage(DoubleSupplier RightVoltage, DoubleSupplier LeftVoltage) {
      return new FunctionalCommand(
        () -> setVoltage(RightVoltage,LeftVoltage), 
        () -> {setVoltage(RightVoltage,LeftVoltage);},
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
        return runVoltage(() -> volts, () -> volts);
    }
    
    public void setVoltage(DoubleSupplier RightVoltage, DoubleSupplier LeftVoltage) {

      leftMotorVoltage = LeftVoltage.getAsDouble()*10;
      rightMotorVoltage = RightVoltage.getAsDouble()*10;

      shooterIO.setVoltage(leftMotorVoltage,rightMotorVoltage);

      isVoltageRightClose(RightVoltage.getAsDouble());
      isVoltageLeftClose(LeftVoltage.getAsDouble());

    }
    
    public void setVoltage(double RightVoltage, double LeftVoltage){
      setVoltage(() -> RightVoltage, () -> LeftVoltage);
    }
    
    public void setRPM(double RightRPM, double LeftRPM) {
        leftSetpointRPM = LeftRPM; 
        rightSetpointRPM = RightRPM;
        shooterIO.setRPM(LeftRPM, RightRPM);
    }

    public void setRPM(DoubleSupplier RightRPM, DoubleSupplier LeftRPM) {
      setRPM(RightRPM.getAsDouble(),LeftRPM.getAsDouble());
    }
    
      public Command stop() {
        return runVoltage(0);
      }
      
      @AutoLogOutput(key = "Shooter/LeftSpeedMetersPerSecond")
      public double getLeftSpeedMetersPerSecond() {
        return shooterInputs.lShooterVelocityRPM * ShooterConstants.wheelRadiusM * 2 * Math.PI / 60;
      }
      
      @AutoLogOutput(key = "Shooter/RightSpeedMetersPerSecond")
      public double getRightSpeedMetersPerSecond() {
        return shooterInputs.rShooterVelocityRPM * ShooterConstants.wheelRadiusM * 2 * Math.PI / 60;
      }
    
      @AutoLogOutput(key = "Shooter/ReadyToShoot")
      public boolean readyToShoot() {
        return shooterInputs.rShooterVelocityRPM > 5_000 && shooterInputs.lShooterVelocityRPM > 5_000;
      }



}
