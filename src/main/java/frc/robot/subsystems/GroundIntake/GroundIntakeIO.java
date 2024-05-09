public interface GroundIntakeIO {
    @Autolog
    public class static GroundIntakeIOInputs {
        
    }
    //Updates the values found in the GroundIntakeIOInputs class (used for logging)
    public default void updateInputs(GroundIntakeIOInputs inputs) {}

    //set the angular velocity of the flywheels
    public default void setVelocityRad(float radians) {}

    //gets the angular velocity of the flywheels
    public default float getVelocityRad() {return 0;}

    //if true, this function will force the flywheel to break
    public default void setBrake(boolean brake) {}

    //sets the PID of the flywheel
    public default void setPID( pid) {}
}