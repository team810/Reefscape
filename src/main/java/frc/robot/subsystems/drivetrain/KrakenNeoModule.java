package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;

public class KrakenNeoModule implements SwerveModuleIO{
    private final SwerveModuleID id;
    private final String idString;

    private final TalonFX driveMotor;
    private final TalonFXSimState driveSimState;
    private final DCMotorSim driveMotorSim;
    private double driveAppliedVoltage;
    private VelocityVoltage driveMotorControl;

    private final SparkMax steerMotor;
    private final PIDController steerController;
    private final CANcoder encoder;
    private final CANcoderSimState encoderSimState;
    private double steerAppliedVoltage;

    private double position;
    private double velocity;
    private double acceleration;
    private double torque;
    private double force;

    private double theta;
    private double omega;

    private Observer.ModuleObservationRaw rawInput;

    private SwerveModuleState targetState;

    public KrakenNeoModule(SwerveModuleID id)
    {
        this.id = id;
        this.idString = this.id.toString();
        if (Robot.isReal()) {
            driveMotor = new TalonFX(DrivetrainConstants.getDriveID(id),DrivetrainConstants.CAN_BUS);
        }else{
            driveMotor = new TalonFX(DrivetrainConstants.getDriveID(id) + 10,DrivetrainConstants.CAN_BUS);
        }
        driveMotor.getConfigurator().apply(DrivetrainConstants.getDriveConfig(id));
        driveMotor.setPosition(0);
        driveSimState = driveMotor.getSimState();

        driveMotorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1),.000000000001,1),DCMotor.getKrakenX60Foc(1));

        steerMotor = new SparkMax(DrivetrainConstants.getSteerID(id), SparkLowLevel.MotorType.kBrushless);
        steerMotor.clearFaults();
        steerMotor.configure(DrivetrainConstants.getSteerConfig(), SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);

        steerAppliedVoltage = 0;

        // We are not relying on really any feedback from the motors so we can set the periodic frames very high to reduces unnecessary canbus usage

        encoder = new CANcoder(DrivetrainConstants.getEncoderID(id), DrivetrainConstants.CAN_BUS);
        
        encoderSimState = encoder.getSimState();

        if (Robot.isReal())
        {
            steerController = new PIDController(
                    DrivetrainConstants.STEER_KP,
                    DrivetrainConstants.STEER_KI,
                    DrivetrainConstants.STEER_KD
            );
        }else{
            steerController = new PIDController(
                    15,
                    0,
                    0
            );
        }

        steerController.enableContinuousInput(-Math.PI, Math.PI);

        driveMotor.optimizeBusUtilization();
        encoder.optimizeBusUtilization();

        rawInput = new Observer.ModuleObservationRaw();
    }

    @Override
    public void readPeriodic(Observer.ModuleObservationRaw data) {
        rawInput = data;
        targetState = new SwerveModuleState();

        Logger.recordOutput("Drivetrain/" + idString + "/" + "VelocityRPS", rawInput.velocity);
        Logger.recordOutput("Drivetrain/" + idString + "/" + "Position", getPosition());
        Logger.recordOutput("Drivetrain/" + idString + "/" + "Velocity", getVelocity());
        Logger.recordOutput("Drivetrain/" + idString + "/" + "Acceleration", getAcceleration());
        Logger.recordOutput("Drivetrain/" + idString + "/" + "DriveAppliedVoltage", rawInput.appliedVoltage);
        Logger.recordOutput("Drivetrain/" + idString + "/" + "DriveSupplyVoltage", rawInput.supplyVoltage);
        Logger.recordOutput("Drivetrain/" + idString + "/" + "DriveSupplyCurrent", rawInput.supplyCurrent);
        Logger.recordOutput("Drivetrain/" + idString + "/" + "DriveAppliedCurrent", rawInput.appliedCurrent);
        Logger.recordOutput("Drivetrain/" + idString + "/" + "Force", getForce());
        Logger.recordOutput("Drivetrain/" + idString + "/" + "Torque", getTorque());

        Logger.recordOutput("Drivetrain/" + idString + "/" + "Theta", getTheta());
        Logger.recordOutput("Drivetrain/" + idString + "/" + "Omega", getOmega());
    }

    @Override
    public void writePeriodic() {
        targetState.optimize(new Rotation2d(getTheta().in(Radians)));
        double velocity = targetState.speedMetersPerSecond;
        velocity = (velocity/(DrivetrainConstants.WHEEL_DIAMETER_METERS * Math.PI)) * DrivetrainConstants.DRIVE_GEAR_RATIO; // converts mps to rotations of motor per second

        // This code was taking up to .019 milliseconds to run
        driveMotorControl = new VelocityVoltage(velocity);
        driveMotorControl.UpdateFreqHz = 1000;
        driveMotorControl.FeedForward = 0;
        driveMotorControl.Slot = 0;
        driveMotorControl.EnableFOC = true;
        driveMotorControl.LimitForwardMotion = false;
        driveMotorControl.LimitReverseMotion = false;
        driveMotorControl.UseTimesync = true;
        driveMotor.setControl(driveMotorControl);

        double targetAngle = MathUtil.angleModulus(targetState.angle.getRadians()); // The angle is rapped from -PI to PI
        if (!DriverStation.isDisabled()) {
            steerAppliedVoltage = steerController.calculate(getTheta().in(Radians),targetAngle);
        }else{
            steerAppliedVoltage = 0;
        }
        if (steerAppliedVoltage != 0) {
            steerAppliedVoltage = steerAppliedVoltage + DrivetrainConstants.STEER_KS;
        }
        steerAppliedVoltage = MathUtil.clamp(steerAppliedVoltage, -12, 12);
        steerMotor.setVoltage(steerAppliedVoltage);
        Logger.recordOutput("Drivetrain/" + idString + "/" + "AppliedState", targetState);
        Logger.recordOutput("Drivetrain/" + idString + "/" + "AppliedVelocityRPM", velocity);
        Logger.recordOutput("Drivetrain/" + idString + "/" + "Target Angle", targetAngle);
        Logger.recordOutput("Drivetrain/" + idString + "/" + "AppliedSteerVoltage", steerAppliedVoltage);
        Logger.recordOutput("Drivetrain/" + idString + "/" + "SteerAppliedCurrent", steerMotor.getOutputCurrent());
    }

    @Override
    public void moduleSim() {
        // Just drive motor
        driveMotorSim.setInputVoltage(driveSimState.getMotorVoltage());
        driveMotorSim.update(Robot.PERIOD);

        driveSimState.setSupplyVoltage(12);

        driveSimState.setRawRotorPosition(driveMotorSim.getAngularPositionRotations()); // This is just in rotations
        driveSimState.setRotorVelocity(driveMotorSim.getAngularVelocityRPM()/60); // Converts RPM to RPS

        // Steer Simulation
        double simOmega = ((steerAppliedVoltage / 12) *  (DrivetrainConstants.MAX_RPM_FOC / 60)) / DrivetrainConstants.STEER_GEAR_RATIO; // rotations per second
        encoderSimState.setSupplyVoltage(12);
        encoderSimState.setVelocity(simOmega);
        encoderSimState.addPosition(simOmega * Robot.PERIOD);
    }


    @Override
    public Distance getPosition() {
        position = rawInput.position.in(Rotations);
        position = position / DrivetrainConstants.DRIVE_GEAR_RATIO;
        position = position * Math.PI * DrivetrainConstants.WHEEL_DIAMETER_METERS;
        return Meters.of(position);
    }


    @Override
    public LinearVelocity getVelocity() {
        velocity = rawInput.velocity.in(RotationsPerSecond);
        velocity = velocity / DrivetrainConstants.DRIVE_GEAR_RATIO;
        velocity = velocity * Math.PI * DrivetrainConstants.WHEEL_DIAMETER_METERS;
        return MetersPerSecond.of(velocity);
    }

    @Override
    public LinearAcceleration getAcceleration() {
        acceleration = rawInput.acceleration.in(RotationsPerSecondPerSecond);
        acceleration = acceleration / DrivetrainConstants.DRIVE_GEAR_RATIO;
        acceleration = acceleration * Math.PI * DrivetrainConstants.WHEEL_DIAMETER_METERS;
        return MetersPerSecondPerSecond.of(acceleration);
    }

    @Override
    public Torque getTorque() {
        torque = DCMotor.getKrakenX60Foc(1).getTorque(rawInput.appliedCurrent.in(Amps));
        torque = torque * DrivetrainConstants.DRIVE_GEAR_RATIO;

        return NewtonMeters.of(torque);
    }

    @Override
    public Force getForce() {
        force = getTorque().in(NewtonMeters) * Units.inchesToMeters(2);
        return Newton.of(force);
    }

    @Override
    public Voltage getDriveAppliedVoltage() {
        driveAppliedVoltage = rawInput.appliedVoltage.in(Volts);
        return Volts.of(driveAppliedVoltage);
    }

    @Override
    public Angle getTheta() {
        return rawInput.theta;
    }

    @Override
    public AngularVelocity getOmega() {
        omega = rawInput.omega.in(RotationsPerSecond);
        omega = omega * 2 * Math.PI; // convert from rotations per second to radians per second
        return RadiansPerSecond.of(omega);
    }

    @Override
    public Voltage getSteerAppliedVoltage() {
        return Volts.of(steerAppliedVoltage);
    }

    @Override
    public SwerveModuleState getCurrentState() {
        return new SwerveModuleState(getVelocity(), new Rotation2d(getTheta()));
    }

    @Override
    public void setTargetState(SwerveModuleState targetState) {
        this.targetState = targetState;
    }

    @Override
    public Observer.ModuleSignals getModuleSignals() {
        return new Observer.ModuleSignals(
                driveMotor.getPosition(),
                driveMotor.getVelocity(),
                driveMotor.getAcceleration(),
                driveMotor.getSupplyCurrent(),
                driveMotor.getStatorCurrent(),
                driveMotor.getSupplyVoltage(),
                driveMotor.getMotorVoltage(),
                encoder.getAbsolutePosition(),
                encoder.getVelocity()
        );
    }
}
