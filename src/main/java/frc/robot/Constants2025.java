package frc.robot;

import java.util.HashMap;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.lib.util.SwerveModuleConstants;

public final class Constants2025 {
    public static double TeleSpeedScale = 0.65;
    public static final double stickDeadband = 0.1;
    public static final String canivore_name = "Canivore5515";
    public static final class Swerve {
        public static final int pigeonID = 0;


        public static final double wheelDiameter = Units.inchesToMeters(4);
        public static final double wheelCircumference = wheelDiameter * Math.PI;        

        public static final double angleGearRatio = (18.75 / 1.0);

        public static final double angleKP = 100.0;
        public static final double angleKI = 0.0;
        public static final double angleKD = 0.0;

        /* Drivetrain Constants */
        public static final double trackWidth = 0.55245; //0.52705;
        public static final double wheelBase = 0.55245; // 0.52705; 

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = 5.36;

        /* Motor Inverts */

        /* Angle Encoder Invert */

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 25;
        public static final int angleCurrentThreshold = 40;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 35;
        public static final int driveCurrentThreshold = 60;
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */

        /* Drive Motor PID Values */
        public static final double driveKP = 1;//TODO
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = 0.08; //TODO
        public static final double driveKV = 1.9429;//TODO
        public static final double driveKA = 0.2987;//TODO
        // public static final double driveKS = 0; //TODO
        // public static final double driveKV = 0;//TODO
        // public static final double driveKA = 0;//TODO

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 3;
        /** Radians per Second */
        public static final double maxAngularVelocity = maxSpeed/trackWidth*1.414; 
        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        private static final class Mod0 {
            public static final int moduleNumber = 0;
            public static final InvertedValue driveMotorInvert = InvertedValue.CounterClockwise_Positive;
            public static final InvertedValue angleMotorInvert = InvertedValue.Clockwise_Positive;
            public static final SensorDirectionValue cancoderInvert = SensorDirectionValue.CounterClockwise_Positive;

            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 9;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(-0.008057);
        }

        /* Front Right Module - Module 1 */
        private static final class Mod1 {
            public static final int moduleNumber = 1;
            public static final InvertedValue driveMotorInvert = InvertedValue.Clockwise_Positive;
            public static final InvertedValue angleMotorInvert = InvertedValue.Clockwise_Positive;
            public static final SensorDirectionValue cancoderInvert = SensorDirectionValue.CounterClockwise_Positive;
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 10;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(-0.352051);
        }
        
        /* Back Left Module - Module 2 */
        private static final class Mod2 {
            public static final int moduleNumber = 2;
            public static final InvertedValue driveMotorInvert = InvertedValue.CounterClockwise_Positive;
            public static final InvertedValue angleMotorInvert = InvertedValue.Clockwise_Positive;
            public static final SensorDirectionValue cancoderInvert = SensorDirectionValue.CounterClockwise_Positive;
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 11;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(0.036377);
        }

        /* Back Right Module - Module 3 */
        private static final class Mod3 {
            public static final int moduleNumber = 3;
            public static final InvertedValue driveMotorInvert = InvertedValue.Clockwise_Positive;
            public static final InvertedValue angleMotorInvert = InvertedValue.Clockwise_Positive;
            public static final SensorDirectionValue cancoderInvert = SensorDirectionValue.CounterClockwise_Positive;
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 12;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(-0.229492);
        }

        public static class Mod {
            public Mod(int moduleNumber, InvertedValue driveMotorInvert, InvertedValue angleMotorInvert, SensorDirectionValue cancoderInvert, int driveMotorID, int angleMotorID, int canCoderID, Rotation2d angleOffset) {
                this.moduleNumber = moduleNumber;
                this.driveMotorInvert = driveMotorInvert;
                this.angleMotorInvert = angleMotorInvert;
                this.cancoderInvert = cancoderInvert;
                this.driveMotorID = driveMotorID;
                this.angleMotorID = angleMotorID;
                this.canCoderID = canCoderID;
                this.angleOffset = angleOffset;
                this.constants = new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
                this.ctreConfigs = new CTREConfigs2025(this);
            }
            public final int moduleNumber;
            public final InvertedValue driveMotorInvert;
            public final InvertedValue angleMotorInvert;
            public final SensorDirectionValue cancoderInvert;
            public final int driveMotorID;
            public final int angleMotorID;
            public final int canCoderID;
            public final Rotation2d angleOffset;
            public final SwerveModuleConstants constants;
            public final CTREConfigs2025 ctreConfigs;
        }
        public static final Mod[] modList = new Mod[] {
            new Mod(Mod0.moduleNumber, Mod0.driveMotorInvert, Mod0.angleMotorInvert, Mod0.cancoderInvert, Mod0.driveMotorID, Mod0.angleMotorID, Mod0.canCoderID, Mod0.angleOffset), 
            new Mod(Mod1.moduleNumber, Mod1.driveMotorInvert, Mod1.angleMotorInvert, Mod1.cancoderInvert, Mod1.driveMotorID, Mod1.angleMotorID, Mod1.canCoderID, Mod1.angleOffset), 
            new Mod(Mod2.moduleNumber, Mod2.driveMotorInvert, Mod2.angleMotorInvert, Mod2.cancoderInvert, Mod2.driveMotorID, Mod2.angleMotorID, Mod2.canCoderID, Mod2.angleOffset), 
            new Mod(Mod3.moduleNumber, Mod3.driveMotorInvert, Mod3.angleMotorInvert, Mod3.cancoderInvert, Mod3.driveMotorID, Mod3.angleMotorID, Mod3.canCoderID, Mod3.angleOffset)
        };
    }

    public static String LIME_LIGHT_ARPIL_TAG_NAME_RIGHT = "limelight-right";
    public static String LIME_LIGHT_ARPIL_TAG_NAME_LEFT = "limelight-left";

    public static HashMap<Long, Pose2d> aimPoses = new HashMap<Long, Pose2d>() {{
        put(17l, new Pose2d(3.651, 2.552, Rotation2d.fromDegrees(60)));
        put(18l, new Pose2d(2.78, 4.025, Rotation2d.fromDegrees(0)));
        put(19l, new Pose2d(3.696, 5.482, Rotation2d.fromDegrees(-60)));
        put(20l, new Pose2d(5.364, 5.482, Rotation2d.fromDegrees(-120)));
        put(21l, new Pose2d(6.191, 4.010, Rotation2d.fromDegrees(180)));
        put(22l, new Pose2d(5.319, 2.552, Rotation2d.fromDegrees(120)));

        put(6l, new Pose2d(13.932, 2.579, Rotation2d.fromDegrees(120)));
        put(7l, new Pose2d(14.755, 4.010, Rotation2d.fromDegrees(180)));
        put(8l, new Pose2d(13.944, 5.437, Rotation2d.fromDegrees(-120)));
        put(9l, new Pose2d(12.246, 5.513, Rotation2d.fromDegrees(-60)));
        put(10l, new Pose2d(11.389, 4.040, Rotation2d.fromDegrees(0)));
        put(11l, new Pose2d(12.261,2.552, Rotation2d.fromDegrees(60)));
    }};

    public static final class TurningArm {
        public static final int motorID = 13;
        public static final int canCoderID = 14;
        public static final String canBusName = "rio";
        public static final double KP = 7;
        public static final double KI = 1.0;
        public static final double KD = 0.1;
        public static final double KV = 0.0; // 0.1
        public static final double KS = 0.0;
        public static final double KA = 0.0;
        public static final double Velocity = 6; // 100.0;
        public static final double Acceleration = 20; // 600.0;
        public static final double Jerk = 0.0;
        public static final double SensorToMechanismRatio = 1.0;
        public static final double RotorToSensorRatio = 44;
        // bigger value means arm is more expanded(clockwise)
        public static final double zeroPos = 0;
        public static final double basePos = 0.4146980;
        public static final double l1Pos = 1.039307;//0.407227;
        public static final double l2Pos = 1.039307;
        public static final double dodgePos =  1.139307;  // the pos that allows elevator to move
        public static final double l3Pos = 1.039307;
        public static final double l4Pos = 0.6;
        public static final double ball1Pos = 4.3;
        public static final double ball2Pos = 4.3;
    }

    public static final class Elevator {
        public static final int primaryMotorID = 15;
        public static final int followerMotorID = 16;
        public static final int canCoderID = 17;
        public static final String canBusName = "rio";
        public static class Up {
            public static final double KP = 5; // 25;  // 5; // 20.0;
            public static final double KI = 0; //3.2; // 0;
            public static final double KD = 0; //0.2;
        }
        public static class Down {
            public static final double KP = 1.5; // 10.0;
            public static final double KI = 0.05;
            public static final double KD = 0;
        }
        public static final double KV = 0;//1;//0.1;
        public static final double KS = 0;//0.05;
        public static final double KA = 0;//0.07;
        public static final double Velocity = 30; // 30;//50;
        public static final double Acceleration = 30;// 100;// 75;
        public static final double Jerk = 0;
        public static final double SensorToMechanismRatio = 1.0;
        public static final double RotorToSensorRatio = 4.8;//11.33;
        // smaller value means higher position
        public static final double zeroPos = 0;
        public static final double basePos = 0;

        public static final double l1Pos = -4.51582; // -1.75;
        public static final double l2Pos = -4.51582;
        public static final double l3Pos = -8.585644;
        public static final double l4Pos = -14.1;
        public static final double ball1Pos = -3.8;
        public static final double ball2Pos = -8;
        public static final double upDodgePos = -6.5;
        public static final double downDodgePos = -3.47;
    }


    public static final class Intake {
        public static final int motorID = 18;
        public static final String canBusName = "rio";
        public static final double KP = 0.5;
        public static final double KI = 0;
        public static final double KD = 0;
        public static final double KS = 0;
        public static final double KV = 0;
        public static final double KA = 0;
        public static final double coralInSpeed = 15;
        public static final double coralInSlowSpeed = 2;
        public static final double coralInReverseSpeed = -3;
        public static final double coralOutSpeed = 30;
        public static final double BallInSpeed = -40;
        public static final double BAllOutSpeed = 15;
    }

    public static final class Candle {
        public static final int candleID = 19;
        public static final String canBusName = "rio";
    }

    public static final class PathPlanner {
        public static final double constraintsSpeed = 3.;
        public static final double constraintsAccel = 3.;
    }
}