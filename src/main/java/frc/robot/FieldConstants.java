package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

public class FieldConstants {

    public static class BlueReef {

        public static final Pose2d F = new Pose2d(3.2094268798828125,4.015262603759766, new Rotation2d(0));
        public static final Pose2d F_LEFT = new Pose2d(2.9,3.98, new Rotation2d(0));
        public static final Pose2d F_RIGHT = new Pose2d(2.8 , 3.65 , new Rotation2d(0));

        public static final Pose2d FL = new Pose2d(3.8588666915893555,5.132298946380615 , Rotation2d.fromDegrees(-60));
        public static final Pose2d FL_LEFT = new Pose2d(3.6 ,5.45, Rotation2d.fromDegrees(-60));
        public static final Pose2d FL_RIGHT = new Pose2d(3.33 ,5.26  , Rotation2d.fromDegrees(-60));

        public static final Pose2d BL = new Pose2d(5.150125980377197, 5.117058753967285 ,Rotation2d.fromDegrees(-120));
        public static final Pose2d BL_LEFT = new Pose2d(5.27 ,5.5, Rotation2d.fromDegrees(-120));
        public static final Pose2d BL_RIGHT = new Pose2d(4.8,5.35 , Rotation2d.fromDegrees(-120));

        public static final Pose2d B = new Pose2d(5.77358865737915,4.0130109786987305, new Rotation2d(3.14));
        public static final Pose2d B_LEFT = new Pose2d(5.870543003082275 ,4.1972222328186035  , new Rotation2d(3.14) );
        public static final Pose2d B_RIGHT = new Pose2d(5.863711833953857 , 3.8693346977233887 , new Rotation2d(3.14));

        public static final Pose2d BR = new Pose2d(5.1501264572143555,2.921952247619629, Rotation2d.fromDegrees(120));
        public static final Pose2d BR_LEFT = new Pose2d(5.36 ,2.63 , Rotation2d.fromDegrees(120));
        public static final Pose2d BR_RIGHT = new Pose2d(5.62 ,2.83  , Rotation2d.fromDegrees(120));

        public static final Pose2d FR = new Pose2d(3.8382577896118164, 2.921952247619629, Rotation2d.fromDegrees(60));
        public static final Pose2d FR_RIGHT = new Pose2d(3.98 , 2.38, Rotation2d.fromDegrees(60));
        public static final Pose2d FR_LEFT = new Pose2d(3.72 ,2.55 , Rotation2d.fromDegrees(60));


//        public static final Pose2d LEFT_SOURCE_TAG = new Pose2d(.851,7.4,Rotation2d.fromDegrees(126));
//        public static final Pose2d RIGHT_SOURCE_TAG = new Pose2d(.851,.6553,Rotation2d.fromDegrees(-126));
//        public static final Distance SOURCE_HORIZONTAL_OFFSET = Inches.of(16);
//        public static final Distance SOURCE_VERTICAL_OFFSET = Inches.of(17.625 + 4.5);

        public static final Pose2d LEFT_SOURCE = new Pose2d(1.24, 7.03, Rotation2d.fromDegrees(126));
        public static final Pose2d RIGHT_SOURCE = new Pose2d(1.28, .96, Rotation2d.fromDegrees(-126));

        public static final Distance LEFT_HORIZONTAL_OFFSET = Meters.of(.19 - .1641);
        public static final Distance RIGHT_HORIZONTAL_OFFSET = Meters.of(.19 + .1641);
        public static final Distance DEPTH_OFFSET = Inches.of(17.625 + 14); // The 17.625 is the size of the robot and the 14 is the distance back from the reef, both are in inches

        // This would be if these are april tag locations
        public static final Pose2d CENTER_AB = new Pose2d(3.6576, 4.0259, Rotation2d.fromDegrees(0));
        public static final Pose2d CENTER_CD = new Pose2d(4.0739,3.3063, Rotation2d.fromDegrees(60));
        public static final Pose2d CENTER_EF = new Pose2d(4.90474,3.3063, Rotation2d.fromDegrees(120));
        public static final Pose2d CENTER_GH = new Pose2d(5.321,4.0259, Rotation2d.fromDegrees(180));
        public static final Pose2d CENTER_IJ = new Pose2d(4.9074, 4.754, Rotation2d.fromDegrees(-120));
        public static final Pose2d CENTER_KL = new Pose2d(4.0739,4.754, Rotation2d.fromDegrees(-60));

        public static final Pose2d REEF_A = transform(CENTER_AB, LEFT_HORIZONTAL_OFFSET,DEPTH_OFFSET);
        public static final Pose2d REEF_B = transform(CENTER_AB, RIGHT_HORIZONTAL_OFFSET,DEPTH_OFFSET);
        public static final Pose2d REEF_C = transform(CENTER_CD, LEFT_HORIZONTAL_OFFSET,DEPTH_OFFSET);
        public static final Pose2d REEF_D = transform(CENTER_CD, RIGHT_HORIZONTAL_OFFSET,DEPTH_OFFSET);
        public static final Pose2d REEF_E = transform(CENTER_EF, LEFT_HORIZONTAL_OFFSET,DEPTH_OFFSET);
        public static final Pose2d REEF_F = transform(CENTER_EF, RIGHT_HORIZONTAL_OFFSET,DEPTH_OFFSET);
        public static final Pose2d REEF_G = transform(CENTER_GH, LEFT_HORIZONTAL_OFFSET,DEPTH_OFFSET);
        public static final Pose2d REEF_H = transform(CENTER_GH, RIGHT_HORIZONTAL_OFFSET,DEPTH_OFFSET);
        public static final Pose2d REEF_I = transform(CENTER_IJ, LEFT_HORIZONTAL_OFFSET,DEPTH_OFFSET);
        public static final Pose2d REEF_J = transform(CENTER_IJ, RIGHT_HORIZONTAL_OFFSET,DEPTH_OFFSET);
        public static final Pose2d REEF_K = transform(CENTER_KL, LEFT_HORIZONTAL_OFFSET,DEPTH_OFFSET);
        public static final Pose2d REEF_L = transform(CENTER_KL, RIGHT_HORIZONTAL_OFFSET,DEPTH_OFFSET);

        /**
         * @param pose2d current pose
         * @param horizontal distance to move by
         * @param vertical distance to move by
         * @return transformed pose
         */
        public static Pose2d transform(Pose2d pose2d, Distance horizontal, Distance vertical) {
            double rotation = pose2d.getRotation().getRadians();
            double rotationMinus = pose2d.getRotation().getRadians() - (Math.PI/2);
            double x = pose2d.getX() + (Math.cos(rotationMinus) * horizontal.in(Meters)) + (-Math.cos(rotation) * vertical.in(Meters));
            double y = pose2d.getY() + (Math.sin(rotationMinus) * horizontal.in(Meters)) + (-Math.sin(rotation) * vertical.in(Meters));

            return new Pose2d(x,y,pose2d.getRotation());
        }
    }
    public static class RedReef {
        public static final Pose2d F = new Pose2d();
        public static final Pose2d F_LEFT = new Pose2d();
        public static final Pose2d F_RIGHT = new Pose2d();

        public static final Pose2d FL = new Pose2d();
        public static final Pose2d FL_LEFT = new Pose2d();
        public static final Pose2d FL_RIGHT = new Pose2d();

        public static final Pose2d BL = new Pose2d();
        public static final Pose2d BL_LEFT = new Pose2d();
        public static final Pose2d BL_RIGHT = new Pose2d();

        public static final Pose2d B = new Pose2d();
        public static final Pose2d B_LEFT = new Pose2d();
        public static final Pose2d B_RIGHT = new Pose2d();

        public static final Pose2d BR = new Pose2d();
        public static final Pose2d BR_LEFT = new Pose2d();
        public static final Pose2d BR_RIGHT = new Pose2d();

        public static final Pose2d FR = new Pose2d();
        public static final Pose2d FR_LEFT = new Pose2d();
        public static final Pose2d FR_RIGHT = new Pose2d();
    }
}
