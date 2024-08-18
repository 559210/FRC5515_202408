package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.1;
    public static final String canivore_name = "Canivore5515";

    public static final class Swerve {
        public static final int pigeonID = 0;


        public static final double wheelDiameter = Units.inchesToMeters(4.0);
        public static final double wheelCircumference = wheelDiameter * Math.PI;        
        /** (150 / 7) : 1 */
        public static final double angleGearRatio = ((150.0 / 7.0) / 1.0);

        public static final double angleKP = 100.0;
        public static final double angleKI = 0.0;
        public static final double angleKD = 0.0;

        public static final InvertedValue driveMotorInvert = InvertedValue.CounterClockwise_Positive;
        public static final InvertedValue driveMotorInvert2 = InvertedValue.Clockwise_Positive;
        public static final InvertedValue angleMotorInvert = InvertedValue.Clockwise_Positive;
        public static final InvertedValue angleMotorInvert2 = InvertedValue.CounterClockwise_Positive;
        public static final SensorDirectionValue cancoderInvert = SensorDirectionValue.CounterClockwise_Positive;

        /* Drivetrain Constants */
        public static final double trackWidth = 0.52705; 
        public static final double wheelBase = 0.52705; 

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = 6.12;

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
        public static final double driveKP = 1.1695;//TODO
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = 0.17201; //TODO
        public static final double driveKV = 2.2429;//TODO
        public static final double driveKA = 0.30202;//TODO

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 2.0;
        /** Radians per Second */
        public static final double maxAngularVelocity = maxSpeed/trackWidth*1.414; 
        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 9;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-131.90);//TODO
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 10;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-170.41);//TODO
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 11;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(29.5);//TODO
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 12;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(59.4);//TODO
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class Intake {
        public static final int intakeFalconMotorID = 13;
        public static final int intakeLeftMotorID = 14;
        public static final int intakeRightMotorID = 15;
        public static final double ForwardSpeed_spark = 0.3;//TODO
        public static final double ReverseSpeed_spark = -0.3;//TODO
        public static final double ForwardSpeed_falcon = 30;//TODO
        public static final double ReverseSpeed_falcon = -30;//TODO
        public static final double KP = 0.5;
        public static final double KI = 0;
        public static final double KD = 0;
        public static final double KS = 0;
        public static final double KV = 0;
        public static final double KA = 0;
    }
    
    public static final class Trigger {
        public static final int triggerMotorID = 23;
        public static final double KP = 0.5;
        public static final double KI = 0.0;
        public static final double KD = 0.0;
        public static final double KV = 0.0;
        public static final double KS = 0.0;
        public static final double KA = 0.0;
        public static double HelpIntake = 30;//TODO
        public static double HelpIntakeReverse = -30;//TODO
        public static double shootout = 60;//TODO
    }

    public static final class Elevator {
        public static final int elevatorMotorID = 25;
        public static final double KP = 0.1;
        public static final double KI = 0.0;
        public static final double KD = 0.0;
        public static final double KV = 0.1;
        public static final double KS = 0.05;
        public static final double KA = 0.07;
        public static final double Velocity = 100.0;
        public static final double Acceleration = 600.0;
        public static final double Jerk = 0.0;
        public static final double Top = 0.0; //TODO
        public static final double Bottom = 0.0; //TODO
    }

    public static final class Shooter {
        public static final int flywheelUpMotorID = 21;
        public static final int flywheelDownMotorID = 22;
        public static final double flywheelKP = 0.5;
        public static final double flywheelKI = 0.0;
        public static final double flywheelKD = 0.0;
        public static final double flywheelKV = 0.0;
        public static final double flywheelKS = 0.0;
        public static final double flywheelKA = 0.0;
        public static double shootingCoasting = 32;
        public static double shootingSpeaker = 60; //TODO
        public static double flywheelTolerance = 0.72;
        public static double AmpUpSpeed = 15; //TODO
        public static double AmpDownSpeed = 60; //TODO
    }

    public static final class Candle {
        public static final int candleID = 24;
        public static final String candleBusName = "rio";
    }

    public static String LIME_LIGHT_NOTE_NAME = "limelight-note";
    public static String LIME_LIGHT_AIM_NAME = "limelight-aim";
}
