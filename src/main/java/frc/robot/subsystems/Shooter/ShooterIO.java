package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

    @AutoLog
    class ShooterIOInputs {
        public double lShooterPositionRotations = 0.0;
        public double rShooterPositionRotations = 0.0;

        public double lShooterVelocityRPM = 0.0;
        public double rShooterVelocityRPM = 0.0;

        public double lShooterAppliedVolts = 0.0;
        public double rShooterAppliedVolts = 0.0;

        public double lShooterOutputCurrent = 0.0;
        public double rShooterOutputCurrent = 0.0;
    }
    
    default void updateInputs(ShooterIOInputs inputs) {}

    default void setLeftRPM(double rpm) {}
    
    default void setRightRPM(double rpm) {}

    default void setRPM(double rpm) {
        setLeftRPM(rpm);
        setRightRPM(rpm);
    }

    default void setLeftVoltage(double voltage) {}

    default void setRightVoltage(double voltage) {}

    default void setVoltage(double voltage){
        setLeftVoltage(voltage);
        setRightVoltage(voltage);
    }

    default void setLeftBreak(boolean Isenabled) {}
    
    default void setRightBreak(boolean Isenabled) {}

    default void setBreak(boolean Isenabled) {
        setLeftBreak(Isenabled);
        setRightBreak(Isenabled);
    }

    default void setLeftPID(double p, double i, double d) {}

    default void setLeftFF(double s, double v, double a) {}
  
    default void setRightPID(double p, double i, double d) {}
  
    default void setRightFF(double s, double v, double a) {}

    default void stop() {}


    
}
