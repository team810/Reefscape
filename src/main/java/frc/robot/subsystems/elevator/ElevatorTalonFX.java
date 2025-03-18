package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;

public class ElevatorTalonFX implements ElevatorIO{
    private final TalonFX leader;
    private TalonFXSimState leaderSim;

    private final TalonFX follower;
    private TalonFXSimState followerSim;

    private final TalonFX followerTwo;

    private final DynamicMotionMagicVoltage control;
    private final Follower followerControl;

    private final StatusSignal<Angle> positionSignal;
    private final StatusSignal<AngularVelocity> velocitySignal;
    private final StatusSignal<AngularAcceleration> accelerationSignal;

    private final StatusSignal<Temperature> leaderTempSignal;
    private final StatusSignal<Voltage> leaderAppliedVoltageSignal;
    private final StatusSignal<Current> leaderSupplyCurrentSignal;
    private final StatusSignal<Current> leaderAppliedCurrentSignal;

    private final StatusSignal<Temperature> followerTempSignal;
    private final StatusSignal<Current> followerCurrentSignal;
    private final StatusSignal<Voltage> followerAppliedVoltageSignal;

    private final StatusSignal<Temperature> followerTempTwo;

    private final ElevatorSim elevatorSim;

    private Distance currentHeight;
    private Angle targetHeight;

    public ElevatorTalonFX() {
        leader = new TalonFX(ElevatorConstants.PRIMARY_MOTOR_ID, "mech");
        leaderSim = leader.getSimState();
        follower = new TalonFX(ElevatorConstants.SECONDARY_MOTOR_ID, "mech");
        followerSim = follower.getSimState();

        followerTwo = new TalonFX(3, "mech");

        TalonFXConfiguration config = new TalonFXConfiguration();
        // Current config
        config.CurrentLimits.SupplyCurrentLimit = 40;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 60;
        if (Robot.isReal())
        {
            config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        }else{
            config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        }
        // Voltage config
        config.Voltage.PeakForwardVoltage = 12;
        config.Voltage.PeakReverseVoltage = -12;
        // Motion config
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
        if (Robot.isReal()) {
            config.Slot0.kP = 2.4;
            config.Slot0.kI = 0;
            config.Slot0.kD = .135;
            config.Slot0.kS = .3;
            config.Slot0.kG = .5;
            config.Slot0.kV = .125;
            config.Slot0.kA = 0.01;

            config.MotionMagic.MotionMagicCruiseVelocity = 20;
            config.MotionMagic.MotionMagicAcceleration = 100;
            config.MotionMagic.MotionMagicJerk = 1000;
        }else{
            config.Slot0.kP = 2.4;
            config.Slot0.kI = 0;
            config.Slot0.kD = .15;
            config.Slot0.kS = .2;
            config.Slot0.kG = .5;
            config.Slot0.kV = .125;
            config.Slot0.kA = 0.01;

            config.MotionMagic.MotionMagicCruiseVelocity = 100;
            config.MotionMagic.MotionMagicAcceleration = 500;
            config.MotionMagic.MotionMagicJerk = 4000;
        }

        leader.getConfigurator().apply(config);
        follower.getConfigurator().apply(config);
        followerTwo.getConfigurator().apply(config);

        positionSignal = leader.getPosition();
        velocitySignal = leader.getVelocity();
        accelerationSignal = leader.getAcceleration();

        leaderTempSignal = leader.getDeviceTemp();
        leaderAppliedVoltageSignal = leader.getMotorVoltage();
        leaderAppliedCurrentSignal = leader.getStatorCurrent();
        leaderSupplyCurrentSignal = leader.getSupplyCurrent();

        followerTempSignal = follower.getDeviceTemp();
        followerCurrentSignal = follower.getSupplyCurrent();
        followerAppliedVoltageSignal = follower.getMotorVoltage();

        followerTempTwo = followerTwo.getDeviceTemp();

        control = new DynamicMotionMagicVoltage(0,20,50,500);
        control.EnableFOC = true;
        control.Slot = 0;
        control.UseTimesync = true;
        control.UpdateFreqHz = 1000;

        followerControl = new Follower(leader.getDeviceID(), false);
        followerControl.UpdateFreqHz = 1000;
        follower.setControl(followerControl);
        followerTwo.setControl(followerControl);

        leader.setPosition(0);

        targetHeight = ElevatorConstants.STORE_CORAL_HEIGHT;
        currentHeight = Inches.of(positionSignal.getValue().in(Rotations) * ElevatorConstants.CONVERSION_FACTOR);

        elevatorSim = new ElevatorSim(
                DCMotor.getKrakenX60Foc(3),
                6,
                Pounds.of(15).in(Kilograms),
                ElevatorConstants.DRUM_RADIUS.in(Meters),
                0,// Min height
                ElevatorConstants.ELEVATOR_MAX_HEIGHT.in(Meters),
                true, // Sim gravity
                .01,
                0.000000001,
                0
        );
    }

    @Override
    public void readPeriodic() {

        StatusSignal.refreshAll(positionSignal,
                velocitySignal,
                leaderAppliedVoltageSignal,
                leaderSupplyCurrentSignal,
                leaderAppliedCurrentSignal,
                leaderTempSignal,
                followerAppliedVoltageSignal,
                followerTempSignal,
                followerCurrentSignal,
                followerAppliedVoltageSignal,
                followerTempSignal,

                followerTempTwo,

                leader.getMotorVoltage()
        );


        currentHeight = Inches.of(positionSignal.getValue().in(Rotations) * ElevatorConstants.CONVERSION_FACTOR);
        Logger.recordOutput("Elevator/VoltageTest",leader.getMotorVoltage().getValue());
        Logger.recordOutput("Elevator/CurrentHeightInches", currentHeight.in(Inches));
        Logger.recordOutput("Elevator/TargetHeightInches", targetHeight.in(Rotations));
        Logger.recordOutput("Elevator/AtTargetHeight", atSetpoint());
        Logger.recordOutput("Elevator/Velocity", velocitySignal.getValue().in(RotationsPerSecond));
        Logger.recordOutput("Elevator/Acceleration", accelerationSignal.getValue().in(RotationsPerSecondPerSecond));

        Logger.recordOutput("Elevator/TargetRaw", control.Position);
        Logger.recordOutput("Elevator/RawEncoder", positionSignal.getValue().in(Rotations));

        Logger.recordOutput("Elevator/Leader/Voltage", leaderAppliedVoltageSignal.getValue());
        Logger.recordOutput("Elevator/Leader/AppliedCurrent", leaderAppliedCurrentSignal.getValue().in(Units.Amps));
        Logger.recordOutput("Elevator/Leader/SupplyCurrent", leaderSupplyCurrentSignal.getValue().in(Amps));
        Logger.recordOutput("Elevator/Leader/Temperature",leaderTempSignal.getValue().in(Celsius));

        Logger.recordOutput("Elevator/Follower/Voltage", followerAppliedVoltageSignal.getValue().in(Units.Volts));
        Logger.recordOutput("Elevator/Follower/Current", followerCurrentSignal.getValue().in(Units.Amps));
        Logger.recordOutput("Elevator/Follower/Temperature",followerTempSignal.getValue().in(Celsius));

        Logger.recordOutput("Elevator/FollowerTwo/Temp", followerTempTwo.getValue().in(Celsius));
    }

    @Override
    public void writePeriodic() {
        control.Position = targetHeight.in(Rotations);

        if (control.Position == ElevatorConstants.STORE_CORAL_HEIGHT.in(Rotations) && positionSignal.getValue().in(Rotations) < ElevatorConstants.STORE_CORAL_HEIGHT.in(Rotations) + .5){
            leader.setControl(new VoltageOut(0));
            follower.setControl(new VoltageOut(0));
            followerTwo.setControl(new VoltageOut(0));
        }else {
            leader.setControl(control);
            follower.setControl(followerControl);
            followerTwo.setControl(followerControl);
        }
    }

    @Override
    public void simulationPeriodic() {
        elevatorSim.setInputVoltage(leaderAppliedVoltageSignal.getValue().in(Units.Volts));
        elevatorSim.update(Robot.PERIOD);

        double heightIn = edu.wpi.first.math.util.Units.metersToInches(elevatorSim.getPositionMeters());
        double velocityIn = edu.wpi.first.math.util.Units.metersToInches(elevatorSim.getVelocityMetersPerSecond());
        leaderSim = leader.getSimState();
        leaderSim.setSupplyVoltage(12);
        leaderSim.setRawRotorPosition(heightIn / 1.09);
        leaderSim.setRotorVelocity(velocityIn/ 1.09);
        followerSim = follower.getSimState();
        followerSim.setSupplyVoltage(12);
        followerSim.setRawRotorPosition(heightIn / ElevatorConstants.CONVERSION_FACTOR);
        followerSim.setRotorVelocity(velocityIn/ ElevatorConstants.CONVERSION_FACTOR);
    }

    @Override
    public void setElevator(Angle targetHeight) {
        this.targetHeight = targetHeight;
        control.Position = targetHeight.in(Rotations);

        if (positionSignal.getValue().in(Rotations) > control.Position) {
            // Moving down
            control.Velocity = 55;
            control.Acceleration = 120;
            control.Jerk = 500;
        }else{
            // Moving up
            control.Velocity = 50;
            control.Acceleration = 120;
            control.Jerk = 500;
        }
    }

    @Override
    public Distance getHeight() {
        return currentHeight;
    }

    @Override
    public boolean atSetpoint() {
        return MathUtil.isNear(targetHeight.in(Rotations),positionSignal.getValue().in(Rotations), .125);
    }
}
