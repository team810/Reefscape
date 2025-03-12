package frc.robot.subsystems.coral;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public interface CoralIO {
    public void setVoltage(Voltage voltage);
    public void setPistonState(DoubleSolenoid.Value value);

    public boolean hasCoral();
    public DoubleSolenoid.Value getPistonState();

    public void readPeriodic();
    public default void writePeriodic() {return;};
    public default void simulationPeriodic() {return;};


}
