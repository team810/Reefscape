package frc.robot.subsystems.coral;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;

public class CoralTalonFX implements CoralIO {
    private final TalonFX motor;
    private final VoltageOut voltageControl;

    private final StatusSignal<Voltage> voltageSignal;
    private final StatusSignal<Voltage> supplyVoltageSignal;
    private final StatusSignal<Temperature> temperaturesSignal;
    private final StatusSignal<Current> appliedCurrentSignal;
    private final StatusSignal<Current> supplyCurrentSignal;

//    private final CANrange sensor;
//    private final StatusSignal<Distance> laserDistance;
//    private final StatusSignal<Boolean> laserIsDetected;

    private final DoubleSolenoid piston;

    public CoralTalonFX() {
        motor = new TalonFX(CoralConstants.MOTOR_ID, CoralConstants.CAN_BUS);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 50;
        config.CurrentLimits.StatorCurrentLimit = 100;
        config.Voltage.PeakForwardVoltage = 12;
        config.Voltage.PeakReverseVoltage = -12;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motor.getConfigurator().apply(config);
        voltageControl = new VoltageOut(0);
        voltageControl.EnableFOC = true;

        voltageSignal = motor.getMotorVoltage();
        supplyCurrentSignal = motor.getSupplyCurrent();
        appliedCurrentSignal = motor.getStatorCurrent();
        temperaturesSignal = motor.getDeviceTemp();

        supplyVoltageSignal = motor.getSupplyVoltage();

//        sensor = new CANrange(CoralConstants.LASER_ID, CoralConstants.CAN_BUS);
//        laserDistance = sensor.getDistance();
//        laserIsDetected = sensor.getIsDetected();

        piston = new DoubleSolenoid(
                PneumaticsModuleType.CTREPCM,
                CoralConstants.PISTON_FWD_CHANNEL,
                CoralConstants.PISTON_REV_CHANNEL
        );


    }

    @Override
    public void readPeriodic() {
        StatusSignal.refreshAll(appliedCurrentSignal, supplyCurrentSignal,temperaturesSignal,voltageSignal, supplyVoltageSignal);
        Logger.recordOutput("Coral/HasCoral", hasCoral());
//        Logger.recordOutput("Coral/RawDistance", laserDistance.getValue());
//        Logger.recordOutput("Coral/IsDetected", laserIsDetected.getValue());

        Logger.recordOutput("Coral/AppliedCurrent", appliedCurrentSignal.getValue());
        Logger.recordOutput("Coral/SupplyCurrent", supplyCurrentSignal.getValue());
        Logger.recordOutput("coral/SupplyVoltage", supplyCurrentSignal.getValue());
        Logger.recordOutput("Coral/MotorTemperatures", temperaturesSignal.getValue().in(Celsius));
        Logger.recordOutput("Coral/MotorVoltage", voltageSignal.getValue());
    }


    @Override
    public void setVoltage(Voltage voltage) {
        voltageControl.Output = voltage.in(Units.Volts);
        // FIXME disabled rn
        motor.setControl(voltageControl);
    }

    @Override
    public void setPistonState(DoubleSolenoid.Value state) {
        piston.set(state);
    }

    @Override
    public boolean hasCoral() {
        return appliedCurrentSignal.getValue().in(Amps) > 95;
    }

    @Override
    public DoubleSolenoid.Value getPistonState() {
        return piston.get();
    }
}
