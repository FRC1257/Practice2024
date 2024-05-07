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

    default void setRPM(double LeftRPM, double RightRPM) {}

    default void setRPM(double RPM){
        setRPM(RPM,RPM);
    }

    default void setVoltage(double RightVoltage, double LeftVoltage){}

    default void setVoltage(double Voltage) {
        setVoltage(Voltage, Voltage);
    }

    default void setLeftBreak(boolean Isenabled) {}
    
    default void setRightBreak(boolean Isenabled) {}

    default void setBreak(boolean Isenabled) {
        setLeftBreak(Isenabled);
        setRightBreak(Isenabled);
    }

    default void setPID(double p, double i, double d) {}

    default void setFF(double s, double v, double a) {}

    default void stop() {}


    
}
