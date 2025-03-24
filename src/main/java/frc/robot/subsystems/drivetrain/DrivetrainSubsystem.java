package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Robot;
import frc.robot.Superstructure;
import frc.robot.lib.AdvancedSubsystem;
import frc.robot.lib.LimelightHelpers;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;

import static edu.wpi.first.units.Units.Radian;

public class DrivetrainSubsystem extends AdvancedSubsystem {
    private static DrivetrainSubsystem instance;

    private final SwerveModuleIO frontLeft;
    private final SwerveModuleIO frontRight;
    private final SwerveModuleIO backLeft;
    private final SwerveModuleIO backRight;

    private final Pigeon2 gyro;
    private final Pigeon2SimState gyroSimState;

    private final Observer observer;
    private final SwerveDriveKinematics kinematics;
    private final SwerveDrivePoseEstimator odometry;

    // Any speeds that come in are either going to be robot relative or fully filed relative
    private ChassisSpeeds velocityFOC;
    private ChassisSpeeds velocityRR;
    private final VelocityThetaControlFOC velocityThetaControlFOC;
    private ControlMethods controlMethod;

    private ChassisSpeeds targetSpeed;

    private SwerveModuleState[] targetStates;
    private SwerveModuleState[] currentStates;

    private Pose2d visionPose = new Pose2d();
    private Pose2d targetPose;
    private double xSetpoint;
    private double ySetpoint;
    private double thetaSetpoint;
    private double xOutput;
    private double yOutput;
    private double thetaOutput;
    private boolean xAtSetpoint;
    private boolean yAtSetpoint;
    private boolean thetaAtSetpoint;
    private double xFeedforward;
    private double yFeedForward;
    private double thetaFeedforward;
    private boolean positionalControlEnabled;



    private DrivetrainSubsystem() {

        frontLeft = new KrakenNeoModule(SwerveModuleID.FrontLeft);
        frontRight = new KrakenNeoModule(SwerveModuleID.FrontRight);
        backLeft = new KrakenNeoModule(SwerveModuleID.BackLeft);
        backRight = new KrakenNeoModule(SwerveModuleID.BackRight);

        gyro = new Pigeon2(DrivetrainConstants.GYRO_ID, DrivetrainConstants.CAN_BUS);
//        gyro.getConfigurator().apply(DrivetrainConstants.getGyroConfig());

        gyroSimState = gyro.getSimState();
        gyro.reset();

        observer = new Observer(
                frontLeft.getModuleSignals(),
                frontRight.getModuleSignals(),
                backLeft.getModuleSignals(),
                backRight.getModuleSignals(),
                gyro.getYaw()
        );
        observer.start();

        kinematics = DrivetrainConstants.getKinematics();

        Observer.SwerveObservation observation = observer.getObservations().get(0);
        observer.clearObservations();

        odometry = new SwerveDrivePoseEstimator(
                kinematics,
                gyro.getRotation2d(),
                new SwerveModulePosition[]{observation.frontLeft, observation.frontRight, observation.backLeft, observation.backRight},
                new Pose2d(0,0 , new Rotation2d(0))
        );

        targetSpeed = new ChassisSpeeds(0,0,0);
        velocityFOC = new ChassisSpeeds(0,0,0);
        velocityRR = new ChassisSpeeds(0,0,0);
        velocityThetaControlFOC = new VelocityThetaControlFOC();
        controlMethod = ControlMethods.off;

        if (DrivetrainConstants.USING_VISION) {
            // Change the camera pose relative to robot center (x forward, y left, z up, degrees)
            LimelightHelpers.setCameraPose_RobotSpace(DrivetrainConstants.LIME_LIGHT_CORAL,.276, .0127,.2667,0,0,0);
            LimelightHelpers.setCameraPose_RobotSpace(DrivetrainConstants.LIME_LIGHT_SOURCE,-.2476,.107,.9779,0,50,0);
            LimelightHelpers.setCameraPose_RobotSpace(DrivetrainConstants.LIME_LIGHT_REEFG, .295,.066,.635,0,-23,50);
        }

        targetPose = new Pose2d();
        xSetpoint = 0;
        ySetpoint = 0;
        thetaSetpoint = 0;
        xOutput = 0;
        yOutput = 0;
        thetaOutput = 0;
        xAtSetpoint = false;
        yAtSetpoint = false;
        thetaAtSetpoint = false;
        xFeedforward = 0;
        yFeedForward = 0;
        thetaFeedforward = 0;
        positionalControlEnabled = false;
    }

    private void addVision(String cam) {



        if (Robot.isReal() && DrivetrainConstants.USING_VISION && !DriverStation.isDisabled())
        {
            boolean reject = false;
            LimelightHelpers.PoseEstimate results;
            LimelightHelpers.SetRobotOrientation(cam, odometry.getEstimatedPosition().getRotation().getDegrees(),0,0, 0, 0, 0);
            results = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cam);

            if (results != null) {

                if (results.avgTagArea > .01)
                {
                    if(Math.abs(getRate().in(edu.wpi.first.units.Units.RadiansPerSecond)) > DrivetrainConstants.MAX_ANGULAR_VELOCITY_ACCEPT_VISION_DATA)
                    {
                        reject = true;
                    }
                    if(results.tagCount == 0)
                    {
                        reject = true;
                    }
                    if(!reject)
                    {
                        visionPose = results.pose;
                        var xyStds = 1.5;

                        xyStds = Math.sqrt((1.0/2.0 * (results.avgTagArea + .25))) - .15;

//                        if (dist < 2.0) {
//
//                        }

                        if (results.tagCount >= 2) {
                            xyStds *= .4;
                        }


                        odometry.setVisionMeasurementStdDevs(VecBuilder.fill(xyStds, xyStds,1800.0));

                        odometry.addVisionMeasurement(visionPose, results.timestampSeconds);


                    }
                }
            }

        }
    }
    @Override
    public void readPeriodic() {
        var moduleObservations = observer.getModuleObservations();
        frontLeft.readPeriodic(moduleObservations[0]);
        frontRight.readPeriodic(moduleObservations[1]);
        backLeft.readPeriodic(moduleObservations[2]);
        backRight.readPeriodic(moduleObservations[3]);

        addVision(DrivetrainConstants.LIME_LIGHT_REEFG);
        addVision(DrivetrainConstants.LIME_LIGHT_CORAL);
        addVision(DrivetrainConstants.LIME_LIGHT_SOURCE);

        ArrayList<Observer.SwerveObservation> observations = observer.getObservations();

        for (int i = 0; i < observations.size(); i++) {
            odometry.updateWithTime(
                    observations.get(i).timestamp,
                    observations.get(i).yaw,
                    new SwerveModulePosition[]{
                            observations.get(i).frontLeft,
                            observations.get(i).frontRight,
                            observations.get(i).backLeft,
                            observations.get(i).backRight
                    });
        }
        observer.clearObservations();

        currentStates = new SwerveModuleState[]{frontLeft.getCurrentState(), frontRight.getCurrentState(), backLeft.getCurrentState(), backRight.getCurrentState()};

        Logger.recordOutput("RobotPose", getPose());
        Logger.recordOutput("Drivetrain/CurrentModuleStates", currentStates);
        Logger.recordOutput("Drivetrain/CurrentSpeeds", getCurrentSpeeds());
    }

    @Override
    public void writePeriodic() {
        switch (controlMethod) {
            case off -> {
                targetSpeed = new ChassisSpeeds(0,0,0);
            }
            case VelocityFOC -> {
                targetSpeed = velocityFOC;
                targetSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(targetSpeed, DrivetrainSubsystem.getInstance().getPose().getRotation());
            }
            case VelocityRR -> {
                targetSpeed = velocityRR;
            }
            case VelocityThetaControlFOC -> {
                targetSpeed = velocityThetaControlFOC.getTargetSpeeds(odometry.getEstimatedPosition().getRotation());
            }
        }
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(targetSpeed);
        targetStates = states;

        frontLeft.setTargetState(states[0]);
        frontRight.setTargetState(states[1]);
        backLeft.setTargetState(states[2]);
        backRight.setTargetState(states[3]);

        frontLeft.writePeriodic();
        frontRight.writePeriodic();
        backLeft.writePeriodic();
        backRight.writePeriodic();

        Logger.recordOutput("Drivetrain/TargetSpeeds", targetSpeed);
        Logger.recordOutput("Drivetrain/TargetModuleStates", targetStates);

        Logger.recordOutput("Drivetrain/PositionalControl/x/setpoint", xSetpoint);
        Logger.recordOutput("Drivetrain/PositionalControl/x/current", getPose().getX());
        Logger.recordOutput("Drivetrain/PositionalControl/x/atSetpoint", xAtSetpoint);
        Logger.recordOutput("Drivetrain/PositionalControl/x/output", xOutput);
        Logger.recordOutput("Drivetrain/PositionalControl/x/ff", xFeedforward);

        Logger.recordOutput("Drivetrain/PositionalControl/y/setpoint", ySetpoint);
        Logger.recordOutput("Drivetrain/PositionalControl/y/current", getPose().getY());
        Logger.recordOutput("Drivetrain/PositionalControl/y/atSetpoint", yAtSetpoint);
        Logger.recordOutput("Drivetrain/PositionalControl/y/output", yOutput);
        Logger.recordOutput("Drivetrain/PositionalControl/y/ff", yFeedForward);

        Logger.recordOutput("Drivetrain/PositionalControl/theta/setpoint", thetaSetpoint);
        Logger.recordOutput("Drivetrain/PositionalControl/theta/current", getPose().getRotation().getRadians());
        Logger.recordOutput("Drivetrain/PositionalControl/theta/atSetpoint", thetaAtSetpoint);
        Logger.recordOutput("Drivetrain/PositionalControl/theta/output", thetaOutput);
        Logger.recordOutput("Drivetrain/PositionalControl/theta/ff", thetaFeedforward);

        Logger.recordOutput("Drivetrain/PositionalControl/targetPose", targetPose);
        Logger.recordOutput("Drivetrain/positionalControl", positionalControlEnabled);


    }

    @Override
    public void simulationPeriodic() {

        if (Robot.isSimulation()) {
            gyroSimState.addYaw(Units.radiansToDegrees(kinematics.toChassisSpeeds(frontLeft.getCurrentState(),frontRight.getCurrentState(),backLeft.getCurrentState(),backRight.getCurrentState()).omegaRadiansPerSecond * Robot.PERIOD));
        }
    
        frontLeft.moduleSim();
        frontRight.moduleSim();
        backLeft.moduleSim();
        backRight.moduleSim();
    }

    public Measure<AngularVelocityUnit> getRate() {
        return gyro.getAngularVelocityZWorld().getValue();
    }

    public SwerveModuleState[] getTargetStates() {
        return targetStates;
    }

    public SwerveModuleState[] getCurrentStates() {
        return currentStates;
    }

    public Pose2d getPose() {
        return odometry.getEstimatedPosition();
    }

    // This is for logging and debugging
    public void setTargetPoseLog(Pose2d targetPose, double xSetpoint, double ySetpoint, double thetaSetpoint, double xOutput, double yOutput, double thetaOutput, boolean xAtSetpoint, boolean yAtSetpoint, boolean thetaAtSetpoint) {
        this.targetPose = targetPose;
        this.xSetpoint = xSetpoint;
        this.ySetpoint = ySetpoint;
        this.thetaSetpoint = thetaSetpoint;
        this.xOutput = xOutput;
        this.yOutput = yOutput;
        this.thetaOutput = thetaOutput;
        this.xAtSetpoint = xAtSetpoint;
        this.yAtSetpoint = yAtSetpoint;
        this.thetaAtSetpoint = thetaAtSetpoint;
    }
    public void setTargetPoseLog(Pose2d targetPose, double xSetpoint, double ySetpoint, double thetaSetpoint, double xOutput, double yOutput, double thetaOutput, boolean xAtSetpoint, boolean yAtSetpoint, boolean thetaAtSetpoint, double xFeedforward, double yFeedForward, double thetaFeedforward)
    {
        setTargetPoseLog(targetPose,xSetpoint,ySetpoint,thetaSetpoint,xOutput,yOutput,thetaOutput,xAtSetpoint,yAtSetpoint,thetaAtSetpoint);
        this.xFeedforward = xFeedforward;
        this.yFeedForward = yFeedForward;
        this.thetaFeedforward = thetaFeedforward;
    }

    public void setPositionalControl(boolean enabled) {
        positionalControlEnabled = enabled;
    }

    public void resetGyro() {
        if (Superstructure.getInstance().getAlliance() == DriverStation.Alliance.Blue)
        {
            resetPose(new Pose2d(getPose().getX(), getPose().getY(), Rotation2d.fromRadians(0)));
        }else{
            resetPose(new Pose2d(getPose().getX(), getPose().getY(), Rotation2d.fromRadians(-Math.PI)));
        }
        setImuMode(0);
        resetLLGyro();
        setImuMode(0);
    }

    public void switchAlliances() {
        if (Superstructure.getInstance().getAlliance() == DriverStation.Alliance.Blue)
        {
            resetPose(new Pose2d(getPose().getX(), getPose().getY(), Rotation2d.fromRadians(0)));
        }else{
            resetPose(new Pose2d(getPose().getX(), getPose().getY(), Rotation2d.fromRadians(-Math.PI)));
        }
    }

    public void resetPose(Pose2d pose) {
        odometry.resetPosition(gyro.getRotation2d(), new SwerveModulePosition[]{
                new SwerveModulePosition(frontLeft.getPosition(),Rotation2d.fromRadians(frontLeft.getTheta().in(Radian))),
                new SwerveModulePosition(frontRight.getPosition(),Rotation2d.fromRadians(frontRight.getTheta().in(Radian))),
                new SwerveModulePosition(backLeft.getPosition(),Rotation2d.fromRadians(backLeft.getTheta().in(Radian))),
                new SwerveModulePosition(backRight.getPosition(),Rotation2d.fromRadians(backRight.getTheta().in(Radian)))},
                pose);
    }

    public void setControlMode(ControlMethods control) {
        this.controlMethod = control;
    }

    public void setVelocityFOC(ChassisSpeeds targetSpeed) {
        this.velocityFOC = targetSpeed;
    }

    public void setVelocityThetaControlFOC(double horizontalSpeed, double verticalSpeed, Rotation2d targetAngle, boolean isAutoLock) {
        velocityThetaControlFOC.setControl(horizontalSpeed, verticalSpeed, targetAngle, isAutoLock);
    }

    /**
     * @return The current classifieds calculated from sensor data
     */
    public ChassisSpeeds getCurrentSpeeds() {
        return kinematics.toChassisSpeeds(frontLeft.getCurrentState(), frontRight.getCurrentState(), backLeft.getCurrentState(), backRight.getCurrentState());
    }
    public ChassisSpeeds getVelocityRR() {
        return velocityRR;
    }

    public void setVelocityRR(ChassisSpeeds velocityRR) {
        this.velocityRR = velocityRR;
    }

    private static class VelocityThetaControlFOC {
        private final PIDController thetaController;
        private double horizontalSpeed = 0;
        private double verticalSpeed = 0;
        private Rotation2d targetAngle = new Rotation2d();

        private boolean isThetaLock;

        public VelocityThetaControlFOC() {
            thetaController = new PIDController(
                    10,
                    0,
                    0
            );
            thetaController.enableContinuousInput(-Math.PI, Math.PI);
            thetaController.setTolerance(Math.toRadians(.01));
            isThetaLock = false;
        }

        public void setControl(double horizontalSpeed, double verticalSpeed, Rotation2d targetAngle, boolean isThetaLock) {
            this.horizontalSpeed = horizontalSpeed;
            this.verticalSpeed = verticalSpeed;
            this.targetAngle = targetAngle;
            this.isThetaLock = isThetaLock;
        }
        public ChassisSpeeds getTargetSpeeds(Rotation2d currentAngle) {
            double omega = thetaController.calculate(currentAngle.getRadians(), targetAngle.getRadians());
            omega = MathUtil.clamp(omega, -10,10);
            omega = MathUtil.applyDeadband(omega,.01);
//            if (isThetaLock && horizontalSpeed == 0 && verticalSpeed == 0) {
//                omega = 0;
//            }
            ChassisSpeeds speeds = new ChassisSpeeds(horizontalSpeed, verticalSpeed, omega);
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, DrivetrainSubsystem.getInstance().getPose().getRotation());
            return speeds;
        }

    }

    public enum ControlMethods {
        off,
        VelocityFOC, // Velocity control filed relative
        VelocityRR, // Velocity control robot relative
        VelocityThetaControlFOC,
    }

    public static DrivetrainSubsystem getInstance() {
        if (instance == null) {
            instance = new DrivetrainSubsystem();
        }
        return instance;
    }

    public void resetLLGyro() {
        // This will be getting called periodically while disabled and when the reset gyro button is pressed
        LimelightHelpers.SetRobotOrientation(DrivetrainConstants.LIME_LIGHT_SOURCE, odometry.getEstimatedPosition().getRotation().getDegrees(), 0,0,0,0,0);
        LimelightHelpers.SetRobotOrientation(DrivetrainConstants.LIME_LIGHT_CORAL, odometry.getEstimatedPosition().getRotation().getDegrees(), 0,0,0,0,0);
//        LimelightHelpers.SetRobotOrientation(DrivetrainConstants.LIME_LIGHT_ALGAE, odometry.getEstimatedPosition().getRotation().getDegrees(),0,0,0,0,0);

    }

    /**
     * - Use external IMU yaw submitted via SetRobotOrientation() for MT2 localization. The internal IMU is ignored entirely.
    * 1 - Use external IMU yaw submitted via SetRobotOrientation(), and configure the LL4 internal IMU's fused yaw to match the submitted yaw value.
     * 2 - Use internal IMU for MT2 localization.
     */
    public void setImuMode(int mode) {
        LimelightHelpers.SetIMUMode(DrivetrainConstants.LIME_LIGHT_SOURCE, mode);
        LimelightHelpers.SetIMUMode(DrivetrainConstants.LIME_LIGHT_CORAL, mode);

    }
}
