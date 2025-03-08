package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

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
        public static final Pose2d FR_LEFT = new Pose2d(3.72 ,2.6 , Rotation2d.fromDegrees(60));

        public static final Pose2d LEFT_SOURCE = new Pose2d(1.24, 7.03, Rotation2d.fromDegrees(126));
        public static final Pose2d RIGHT_SOURCE = new Pose2d(1.09, 1.16, Rotation2d.fromDegrees(-126));
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
