package frc.robot.subsystems.drivetrain;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.littletonrobotics.junction.Logger;

public class DrivetrainControl {
    public enum ControlMode {
        FieldCentricTheta,
        FieldCentricOmega,

        RobotCentricTheta,
        RobotCentricOmega,

        PositionControl,
        TrajectoryControl,

        Off,
    }

    public interface ControlIO {
        public ChassisSpeeds generateSpeeds(Pose2d currentPose);
        public ControlMode getControlMode();
        public void logging();
    }

    public class TrajectoryControl implements ControlIO {
        private Pose2d targetPose;

        private double xVelocityFF;
        private double yVelocityFF;
        private double omegaFF;

        private double xControllerOutput;
        private double yControllerOutput;
        private double thetaControllerOutput;

        private double timeStamp;

        public TrajectoryControl(SwerveSample sample) {

        }
    }

    private ControlMode controlMode;
    private ControlIO control;

    public DrivetrainControl(ControlIO control) {
        controlMode = control.getControlMode();
        this.control = control;
    }

    public DrivetrainControl() {
        setControl(ControlMode.Off);
    }

    public DrivetrainControl(ControlMode mode) {
        setControl(mode);
    }

    public void setControl(ControlIO control) {
        this.control = control;
        controlMode = control.getControlMode();
    }

    public void setControl(ControlMode mode) {
        controlMode = mode;
        switch(controlMode) {
            case FieldCentricTheta -> {
            }
            case FieldCentricOmega -> {
            }
            case RobotCentricTheta -> {
            }
            case RobotCentricOmega -> {
            }
            case PositionControl -> {
            }
            case TrajectoryControl -> {
            }
            case Off -> {
                control = null;
            }
        }
    }

    public void logging() {
        Logger.recordOutput("Drivetrain/Mode", controlMode);

        if (controlMode != ControlMode.Off) {
            control.logging();
        }
    }

    public ChassisSpeeds getSpeeds(Pose2d currentPose) {
        if (controlMode == ControlMode.Off) {
            return new ChassisSpeeds(0,0,0);
        }else{
            return control.generateSpeeds(currentPose);
        }
    }
}
