package frc.robot.subsystems.algae;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.UpdateModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;

// CCW+ 0 is horizontal to the ground, 90 is straight up
public class AlgaeTalonFX implements AlgaeIO {
    private final TalonFX pivotMotor;
    private final PositionVoltage pivotControl;
    private final SingleJointedArmSim pivotSim;
    private TalonFXSimState pivotSimState;

    private final StatusSignal<Angle> pivotPositionSignal;
    private final StatusSignal<AngularVelocity> pivotVelocitySignal;
    private final StatusSignal<Voltage> pivotVoltageSignal;
    private final StatusSignal<Temperature> pivotTemperatureSignal;
    private final StatusSignal<Current> pivotAppliedCurrentSignal;

    private Angle targetPivot;

    private final TalonFX driveMotor;
    private VoltageOut driveVoltageControl;
    private Voltage driveAppliedVoltage;

    private final StatusSignal<Voltage> driveVoltageSignal;
    private final StatusSignal<Temperature> driveTemperatureSignal;
    private final StatusSignal<Current> driveAppliedCurrentSignal;
    private final StatusSignal<Current> driveSupplyCurrentSignal;

    private final CANrange laser;
    private final StatusSignal<Distance> distanceSignal;
    private final StatusSignal<Boolean> detectedSignal;

    public AlgaeTalonFX() {
        pivotMotor = new TalonFX(AlgaeConstants.PIVOT_MOTOR_ID, AlgaeConstants.CANBUS);
        TalonFXConfiguration pivotMotorConfig = new TalonFXConfiguration();
        pivotMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        pivotMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        pivotMotorConfig.CurrentLimits.SupplyCurrentLimit = 20;
        pivotMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        pivotMotorConfig.CurrentLimits.StatorCurrentLimit = 80;
        pivotMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        pivotMotorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine; // Takes the cos of the angle and then calculates the input needed to overcome gravity
        pivotMotorConfig.Slot0.kG = .2;
        pivotMotorConfig.Slot0.kV = .125;
        pivotMotorConfig.Slot0.kA = .01;
        pivotMotorConfig.Slot0.kP = 35;
        pivotMotorConfig.Slot0.kI = .1;
        pivotMotorConfig.Slot0.kD = 0;

        pivotMotorConfig.Feedback.SensorToMechanismRatio = 64; // 64 to 1 gear box

        pivotMotorConfig.Voltage.PeakForwardVoltage = 8;
        pivotMotorConfig.Voltage.PeakReverseVoltage = -8;

        pivotMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 100;
        pivotMotorConfig.MotionMagic.MotionMagicAcceleration = 400;
        pivotMotorConfig.MotionMagic.MotionMagicJerk = 1000;

//        pivotMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
//        pivotMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = AlgaeConstants.MAX_PIVOT.in(Units.Rotations);
//        pivotMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
//        pivotMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = AlgaeConstants.MIN_PIVOT.in(Units.Rotations);

        pivotMotor.getConfigurator().apply(pivotMotorConfig);
        targetPivot = AlgaeConstants.STORED_ANGLE;

        pivotControl = new PositionVoltage(AlgaeConstants.STORED_ANGLE);
        pivotControl.EnableFOC = true;
        pivotControl.Slot = 0;
        pivotControl.LimitForwardMotion = false;
        pivotControl.LimitReverseMotion = false;
        pivotControl.UpdateFreqHz = 1000;

        pivotPositionSignal = pivotMotor.getPosition();
        pivotVelocitySignal = pivotMotor.getVelocity();
        pivotTemperatureSignal = pivotMotor.getDeviceTemp();
        pivotVoltageSignal = pivotMotor.getMotorVoltage();
        pivotAppliedCurrentSignal = pivotMotor.getSupplyCurrent();

        pivotSim = new SingleJointedArmSim(
                DCMotor.getKrakenX60Foc(1),
                64,
                .148,
                edu.wpi.first.math.util.Units.inchesToMeters(6),
                AlgaeConstants.MIN_PIVOT.in(Radians),
                AlgaeConstants.MAX_PIVOT.in(Radians),
                true,
                AlgaeConstants.STARTING_ANGLE.in(Radians)
        );
        pivotSim.setState(AlgaeConstants.STARTING_ANGLE.in(Radians),0);
        if (Robot.isReal()) {
            pivotMotor.setPosition(AlgaeConstants.STARTING_ANGLE.in(Rotations));
        }

        driveMotor = new TalonFX(AlgaeConstants.DRIVE_MOTOR_ID, AlgaeConstants.CANBUS);
        TalonFXConfiguration driveMotorConfig = new TalonFXConfiguration();
        driveMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        driveMotorConfig.CurrentLimits.SupplyCurrentLimit = 80;
        driveMotorConfig.CurrentLimits.StatorCurrentLimit = 160;
        driveMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        driveMotorConfig.Voltage.PeakForwardVoltage = 12;
        driveMotorConfig.Voltage.PeakReverseVoltage = -12;
        driveMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveMotor.getConfigurator().apply(driveMotorConfig);

        driveVoltageControl = new VoltageOut(0);
        driveVoltageControl.EnableFOC = false;
        driveVoltageControl.UpdateFreqHz = 1000;
        driveVoltageControl.UseTimesync = false;
        driveVoltageControl.LimitReverseMotion = false;
        driveVoltageControl.LimitForwardMotion = false;

        driveAppliedVoltage = Volts.of(0);

        driveVoltageSignal = driveMotor.getMotorVoltage();
        driveTemperatureSignal = driveMotor.getDeviceTemp();
        driveAppliedCurrentSignal = driveMotor.getStatorCurrent();
        driveSupplyCurrentSignal = driveMotor.getSupplyCurrent();

        laser = new CANrange(AlgaeConstants.LASER_ID, AlgaeConstants.CANBUS);
        CANrangeConfiguration laserConfig = new CANrangeConfiguration();
        laserConfig.ToFParams.UpdateMode = UpdateModeValue.ShortRange100Hz;
        laserConfig.FovParams.FOVRangeX = 6.75;
        laserConfig.FovParams.FOVRangeY = 6.75;
        laserConfig.FutureProofConfigs = true;
        laser.getConfigurator().apply(laserConfig);
        distanceSignal = laser.getDistance();
        detectedSignal = laser.getIsDetected();
    }

    @Override
    public void readPeriodic() {
        StatusSignal.refreshAll(
                pivotPositionSignal,
                pivotVelocitySignal,
                pivotTemperatureSignal,
                pivotVoltageSignal,
                pivotAppliedCurrentSignal,

                distanceSignal,
                detectedSignal,

                driveVoltageSignal,
                driveTemperatureSignal,
                driveAppliedCurrentSignal,
                driveSupplyCurrentSignal
        );

        Logger.recordOutput("Algae/Pivot/TargetAngle",pivotControl.Position);
        Logger.recordOutput("Algae/Pivot/Position",pivotPositionSignal.getValue().in(Units.Rotations));
        Logger.recordOutput("Algae/Pivot/Velocity",pivotVelocitySignal.getValue());
        Logger.recordOutput("Algae/Pivot/MotorTemp", pivotTemperatureSignal.getValue());
        Logger.recordOutput("Algae/Pivot/Voltage", pivotVoltageSignal.getValue());
        Logger.recordOutput("Algae/Pivot/AppliedCurrent", pivotAppliedCurrentSignal.getValue());

        Logger.recordOutput("Algae/Laser/RawValue", distanceSignal.getValue().in(Inches));
        Logger.recordOutput("Algae/Laser/DetectedAlgae", hasAlgae());

        Logger.recordOutput("Algae/Drive/Voltage", driveVoltageSignal.getValue());
        Logger.recordOutput("Algae/Drive/Current", driveAppliedCurrentSignal.getValue());
        Logger.recordOutput("Algae/Drive/SupplyCurrent", driveSupplyCurrentSignal.getValue());
        Logger.recordOutput("Algae/Drive/Temperature", driveTemperatureSignal.getValue().in(Celsius));
        Logger.recordOutput("Algae/Drive/AppliedVoltage", driveAppliedVoltage);
    }

    @Override
    public void writePeriodic() {
        pivotControl.UpdateFreqHz = 1000;
        pivotMotor.setControl(pivotControl); // Apply pivot setpoint
        driveMotor.setControl(driveVoltageControl); // Applied drive voltage
    }

    @Override
    public void simulatePeriodic() {
        pivotSim.setInputVoltage(pivotVoltageSignal.getValue().in(Volts));
        pivotSim.update(Robot.defaultPeriodSecs);

        pivotSimState = pivotMotor.getSimState();
        pivotSimState.setSupplyVoltage(12);
        pivotSimState.setRawRotorPosition((pivotSim.getAngleRads()/(2*Math.PI)) * 64);
        pivotSimState.setRotorVelocity((pivotSim.getVelocityRadPerSec()/(2*Math.PI)) * 64);
    }

    @Override
    public boolean hasAlgae() {
        return false;
//        return MathUtil.isNear(
//                AlgaeConstants.LASER_EXPECTED.in(Units.Meters),
//                distanceSignal.getValue().in(Units.Meters),
//                AlgaeConstants.LASER_TOLERANCE.in(Units.Meters)
//        );
    }

    @Override
    public boolean atPivotSetpoint() {
        return MathUtil.isNear(
                targetPivot.in(Radians),
                pivotPositionSignal.getValue().in(Units.Radians),
            AlgaeConstants.PIVOT_TOLERANCE.in(Units.Radians)
        );
    }

    @Override
    public void setTargetPivot(Angle angle) {
        targetPivot = angle;
        pivotControl.Position = angle.in(Rotations);
    }

    @Override
    public void setDriveVoltage(VoltageOut voltage) {
        driveAppliedVoltage = Volts.of(voltage.Output);
        driveVoltageControl = voltage;
    }

    @Override
    public double getCurrentPivot() {
        return pivotPositionSignal.getValue().in(Units.Radians);
    }
}
