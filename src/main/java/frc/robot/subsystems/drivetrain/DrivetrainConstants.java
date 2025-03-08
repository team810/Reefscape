package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.config.SignalsConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

public class DrivetrainConstants {
    public final static String CAN_BUS = "drivetrain";

    public final static int FRONT_LEFT_DRIVE_ID = 1;
    public final static int FRONT_LEFT_STEER_ID = 1;
    public final static int FRONT_LEFT_ENCODER_ID = 5;

    public final static int FRONT_RIGHT_DRIVE_ID = 2;
    public final static int FRONT_RIGHT_STEER_ID = 2;
    public final static int FRONT_RIGHT_ENCODER_ID = 6;

    public final static int BACK_LEFT_DRIVE_ID = 3;
    public final static int BACK_LEFT_STEER_ID = 3;
    public final static int BACK_LEFT_ENCODER_ID = 7;

    public final static int BACK_RIGHT_DRIVE_ID = 4;
    public final static int BACK_RIGHT_STEER_ID = 4;
    public final static int BACK_RIGHT_ENCODER_ID = 8;

    public final static int GYRO_ID = 9;

    public static final double WHEEL_BASE_WIDTH = Units.inchesToMeters(24); // measure of FL wheel to FR wheel or BL wheel to BR wheel
    public static final double WHEEL_BASE_LENGTH = Units.inchesToMeters(24); // measure of FL wheel to BL wheel// or FR wheel to BR wheel

    public static final double STEER_KP = 4.5; // voltage/radians proportion of volts to error in radians
    public static final double STEER_KI = 0;
    public static final double STEER_KD = 0;

    public static final double YAW_KP = 5; // Radians per second / radians

    public static final double STEER_GEAR_RATIO = 12.8;

    public static final double MOMENT_OF_INTRA_DRIVE = .025 / (DrivetrainConstants.DRIVE_GEAR_RATIO * DrivetrainConstants.DRIVE_GEAR_RATIO); // kg * meters^2

    public static final double MAX_RPM_FOC = 5800;
    public static final double DRIVE_GEAR_RATIO = 5.36;
    public static final double WHEEL_DIAMETER_INCHES = 4.0;
    public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(WHEEL_DIAMETER_INCHES);

    public static final double MASS = Units.lbsToKilograms(135); // Robot Mass kg
    public static final double COEFFICIENT_OF_FRICTION = 1; //
    public static final double MAX_TRACTION = (MASS * 9.8) * COEFFICIENT_OF_FRICTION; // Fn * Mu = Max traction in Newtons
    public static final double MAX_THEORETICAL_ACCELERATION = 15; // m/s^2 f=ma f/m = a
    // 5.761
    public static final double MAX_VELOCITY = ((MAX_RPM_FOC / 60)/DRIVE_GEAR_RATIO) * (WHEEL_DIAMETER_METERS * Math.PI); // Meters per second

    public static final double MAX_ANGULAR_VELOCITY = MAX_VELOCITY * (Math.sqrt((WHEEL_BASE_LENGTH * WHEEL_BASE_LENGTH)+(WHEEL_BASE_WIDTH * WHEEL_BASE_WIDTH)) * Math.PI); // Rotations per second
    public static final double MAX_ANGULAR_ACCELERATION = MAX_THEORETICAL_ACCELERATION * (Math.sqrt((WHEEL_BASE_LENGTH * WHEEL_BASE_LENGTH)+(WHEEL_BASE_WIDTH * WHEEL_BASE_WIDTH)) * Math.PI); // Rotations per second squared

    public static final double MAX_ANGULAR_VELOCITY_ACCEPT_VISION_DATA = 2 * Math.PI; // 1 rotation per second
    public static final String LIME_LIGHT_SOURCE = "limelight-source";
    public static final String LIME_LIGHT_ALGAE = "limelight-algae";
    public static final String LIME_LIGHT_CORAL = "limelight-reef";
    public static final boolean USING_VISION = true;

    public static final double DRIVETRAIN_LENGTH = Units.inchesToMeters(29.25);
    public static final double DRIVETRAIN_WIDTH = Units.inchesToMeters(29.25);

    public static SwerveDriveKinematics getKinematics() {
        return new SwerveDriveKinematics(
                new Translation2d(DrivetrainConstants.WHEEL_BASE_WIDTH / 2.0,
                        DrivetrainConstants.WHEEL_BASE_LENGTH / 2.0),
                // Front right
                new Translation2d(DrivetrainConstants.WHEEL_BASE_WIDTH / 2.0,
                        -DrivetrainConstants.WHEEL_BASE_LENGTH / 2.0),
                // Back left
                new Translation2d(-DrivetrainConstants.WHEEL_BASE_WIDTH / 2.0,
                        DrivetrainConstants.WHEEL_BASE_LENGTH / 2.0),
                // Back right
                new Translation2d(-DrivetrainConstants.WHEEL_BASE_WIDTH / 2.0,
                        -DrivetrainConstants.WHEEL_BASE_LENGTH / 2.0)
        );
    }

    public static int getDriveID(SwerveModuleID id) {
        switch (id) {
            case FrontLeft -> {
                return FRONT_LEFT_DRIVE_ID;
            }
            case FrontRight -> {
                return FRONT_RIGHT_DRIVE_ID;
            }
            case BackLeft -> {
                return BACK_LEFT_DRIVE_ID;
            }
            case BackRight -> {
                return BACK_RIGHT_DRIVE_ID;
            }
        }
        return 0;
    }

    public static int getSteerID(SwerveModuleID id) {
        switch (id) {
            case FrontLeft -> {
                return FRONT_LEFT_STEER_ID;
            }
            case FrontRight -> {
                return FRONT_RIGHT_STEER_ID;
            }
            case BackLeft -> {
                return BACK_LEFT_STEER_ID;
            }
            case BackRight -> {
                return BACK_RIGHT_STEER_ID;
            }
        }
        return 0;
    }

    public static int getEncoderID(SwerveModuleID id) {
        switch (id) {
            case FrontLeft -> {
                return FRONT_LEFT_ENCODER_ID;
            }
            case FrontRight -> {
                return FRONT_RIGHT_ENCODER_ID;
            }
            case BackLeft -> {
                return BACK_LEFT_ENCODER_ID;
            }
            case BackRight -> {
                return BACK_RIGHT_ENCODER_ID;
            }
        }
        return 0;
    }

    public static TalonFXConfiguration getDriveConfig(SwerveModuleID module) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        CurrentLimitsConfigs currentLimitConfig = new CurrentLimitsConfigs();
        currentLimitConfig.StatorCurrentLimitEnable = true;
        currentLimitConfig.SupplyCurrentLimitEnable = true;
        currentLimitConfig.StatorCurrentLimit = 40;
        currentLimitConfig.SupplyCurrentLimit = 80;
        config.CurrentLimits = currentLimitConfig;

        VoltageConfigs voltageConfigs = new VoltageConfigs();
        voltageConfigs.PeakForwardVoltage = 12;
        voltageConfigs.PeakReverseVoltage = -12;
        config.Voltage = voltageConfigs;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        Slot0Configs velocityControllerConfig = new Slot0Configs();
        switch (module)
        {
            case FrontLeft -> {
                velocityControllerConfig.kV = 0.124146;
                velocityControllerConfig.kA = 0;
                velocityControllerConfig.kG = 0;
                velocityControllerConfig.kS = .0;
                velocityControllerConfig.kP = .0;
                velocityControllerConfig.kI = 0;
                velocityControllerConfig.kD = 0;
            }
            case FrontRight -> {
                velocityControllerConfig.kV = 0.124146;
                velocityControllerConfig.kA = 0;
                velocityControllerConfig.kG = 0;
                velocityControllerConfig.kS = .0;
                velocityControllerConfig.kP = .0;
                velocityControllerConfig.kI = 0;
                velocityControllerConfig.kD = 0;
            }
            case BackLeft -> {
                velocityControllerConfig.kV = 0.124146;
                velocityControllerConfig.kA = 0;
                velocityControllerConfig.kG = 0;
                velocityControllerConfig.kS = .0;
                velocityControllerConfig.kP = .0;
                velocityControllerConfig.kI = 0;
                velocityControllerConfig.kD = 0;
            }
            case BackRight -> {
                velocityControllerConfig.kV = 0.124146;
                velocityControllerConfig.kA = 0;
                velocityControllerConfig.kG = 0;
                velocityControllerConfig.kS = .0;
                velocityControllerConfig.kP = .0;
                velocityControllerConfig.kI = 0;
                velocityControllerConfig.kD = 0;
            }
        }
        config.Slot0 = velocityControllerConfig;

        FeedbackConfigs feedbackConfig = new FeedbackConfigs();
        feedbackConfig.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        config.Feedback = feedbackConfig;

        MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
        motionMagicConfigs.MotionMagicCruiseVelocity = DrivetrainConstants.MAX_RPM_FOC;
        motionMagicConfigs.MotionMagicJerk = MAX_THEORETICAL_ACCELERATION * .1;

        config.MotionMagic = motionMagicConfigs;

        AudioConfigs audioConfig = new AudioConfigs();
        audioConfig.BeepOnConfig = true;
        audioConfig.AllowMusicDurDisable = true;
        audioConfig.BeepOnBoot = true;
        config.Audio = audioConfig;

        return config;
    }

    public static CANcoderConfiguration getEncoderConfig(CANcoderConfiguration currentConfig) {
        currentConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = .5;
        currentConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

        return currentConfig;
    }

    public static SparkMaxConfig getSteerConfig()
    {
        SparkMaxConfig config = new SparkMaxConfig();

        SignalsConfig signalConfig = new SignalsConfig();

        signalConfig.absoluteEncoderPositionAlwaysOn(false);
        signalConfig.absoluteEncoderVelocityAlwaysOn(false);
        signalConfig.analogPositionAlwaysOn(false);
        signalConfig.analogVelocityAlwaysOn(false);
        signalConfig.iAccumulationAlwaysOn(false);
        signalConfig.externalOrAltEncoderPositionAlwaysOn(false);
        signalConfig.externalOrAltEncoderVelocityAlwaysOn(false);
        signalConfig.primaryEncoderPositionAlwaysOn(false);
        signalConfig.primaryEncoderVelocityAlwaysOn(false);

        config.apply(signalConfig);
        config.voltageCompensation(12);
        config.smartCurrentLimit(40);
        config.idleMode(SparkBaseConfig.IdleMode.kBrake);

        return config;
    }

    public static Pigeon2Configuration getGyroConfig() {
        Pigeon2Configuration config = new Pigeon2Configuration();
        return config;
    }

    public static Measure<DistanceUnit> INTERLOPE_TOLERANCE = Distance.ofBaseUnits(.05, Meters);
    public static Measure<AngleUnit> INTERLOPE_ANGlE_TOLERANCE = Angle.ofBaseUnits(.5,Degrees);
}
