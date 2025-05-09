package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import edu.wpi.first.math.VecBuilder;
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

    private Pose2d visionPose = new Pose2d();
    private final ArrayList<LimelightHelpers.PoseEstimate> visionResults = new ArrayList<>();

    private SwerveModuleState[] targetStates;
    private SwerveModuleState[] currentStates;

    private final DrivetrainControl control;

    private DrivetrainSubsystem() {
        frontLeft = new KrakenNeoModule(SwerveModuleID.FrontLeft);
        frontRight = new KrakenNeoModule(SwerveModuleID.FrontRight);
        backLeft = new KrakenNeoModule(SwerveModuleID.BackLeft);
        backRight = new KrakenNeoModule(SwerveModuleID.BackRight);

        gyro = new Pigeon2(DrivetrainConstants.GYRO_ID, DrivetrainConstants.CAN_BUS);
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

        control = new DrivetrainControl();

        LimelightHelpers.SetIMUMode(DrivetrainConstants.LIME_LIGHT_SOURCE, 0);
        LimelightHelpers.SetIMUMode(DrivetrainConstants.LIME_LIGHT_CORAL, 0);

        if (DrivetrainConstants.USING_VISION) {
            // Change the camera pose relative to robot center (x forward meters, y left meters, z up meters, roll deg, pitch deg, yaw deg)
            LimelightHelpers.setCameraPose_RobotSpace(DrivetrainConstants.LIME_LIGHT_CORAL,.276, .0127,.2667,0,0,0);
            LimelightHelpers.setCameraPose_RobotSpace(DrivetrainConstants.LIME_LIGHT_SOURCE,-.2476,.107,.9779,0,50,0);
            LimelightHelpers.setCameraPose_RobotSpace(DrivetrainConstants.LIME_LIGHT_REEFG, .295,.066,.635,0,-23,50);
        }
    }

    private void addVision(String cam) {
        if (Robot.isReal())
        {
            LimelightHelpers.PoseEstimate results;
            LimelightHelpers.SetRobotOrientation(cam, odometry.getEstimatedPosition().getRotation().getDegrees(),0,0, 0, 0, 0);
            results = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cam);

            if (results != null)
            {
                visionResults.add(results);
            }

            if (DrivetrainConstants.USING_VISION && !DriverStation.isDisabled())
            {
                boolean reject = false;
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

                            xyStds = Math.sqrt((.6 * (results.avgTagArea + .25))) - .15;

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
    }
    @Override
    public void readPeriodic() {
        var moduleObservations = observer.getModuleObservations();
        frontLeft.readPeriodic(moduleObservations[0]);
        frontRight.readPeriodic(moduleObservations[1]);
        backLeft.readPeriodic(moduleObservations[2]);
        backRight.readPeriodic(moduleObservations[3]);

        visionResults.clear();

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
        Pose2d[] estimatedPose = new Pose2d[] {new Pose2d(), new Pose2d(), new Pose2d()};
        for (int i = 0; i < visionResults.size(); i++) {
            estimatedPose[i] = visionResults.get(i).pose;
        }

        Logger.recordOutput("VisionPoses", estimatedPose);
        Logger.recordOutput("RobotPose", getPose());
        Logger.recordOutput("Drivetrain/CurrentModuleStates", currentStates);
        Logger.recordOutput("Drivetrain/CurrentSpeeds", getCurrentSpeeds());
    }
    @Override
    public void writePeriodic() {
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(control.getSpeeds(getPose()));
        targetStates = states;

        frontLeft.setTargetState(states[0]);
        frontRight.setTargetState(states[1]);
        backLeft.setTargetState(states[2]);
        backRight.setTargetState(states[3]);

        frontLeft.writePeriodic();
        frontRight.writePeriodic();
        backLeft.writePeriodic();
        backRight.writePeriodic();
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

    public void setControl(DrivetrainControl.ControlIO target) {
        this.control.setControl(target);
    }
    public void setControl(DrivetrainControl.ControlMode mode) {
        this.control.setControl(mode);
    }

    public Measure<AngularVelocityUnit> getRate() {
        return gyro.getAngularVelocityZWorld().getValue();
    }
    public ChassisSpeeds getCurrentSpeeds() {return kinematics.toChassisSpeeds(frontLeft.getCurrentState(), frontRight.getCurrentState(), backLeft.getCurrentState(), backRight.getCurrentState());}
    public SwerveModuleState[] getCurrentStates() {
        return currentStates;
    }
    public Pose2d getPose() {
        return odometry.getEstimatedPosition();
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
    public void resetGyro() {
        if (Superstructure.getInstance().getAlliance() == DriverStation.Alliance.Blue)
        {
            resetPose(new Pose2d(getPose().getX(), getPose().getY(), Rotation2d.fromRadians(0)));
        }else{
            resetPose(new Pose2d(getPose().getX(), getPose().getY(), Rotation2d.fromRadians(-Math.PI)));
        }
        LimelightHelpers.SetRobotOrientation(DrivetrainConstants.LIME_LIGHT_SOURCE, odometry.getEstimatedPosition().getRotation().getDegrees(), 0,0,0,0,0);
        LimelightHelpers.SetRobotOrientation(DrivetrainConstants.LIME_LIGHT_CORAL, odometry.getEstimatedPosition().getRotation().getDegrees(), 0,0,0,0,0);
    }
    public static DrivetrainSubsystem getInstance() {
        if (instance == null) {
            instance = new DrivetrainSubsystem();
        }
        return instance;
    }
}
