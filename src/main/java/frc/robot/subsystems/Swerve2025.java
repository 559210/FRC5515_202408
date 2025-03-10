package frc.robot.subsystems;
import frc.robot.SwerveModule2025;
import frc.robot.Constants2025;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;
import frc.robot.StateController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;


import static edu.wpi.first.units.Units.*;

import java.util.concurrent.ThreadPoolExecutor.DiscardOldestPolicy;

public class Swerve2025 extends SubsystemBase {
    static final boolean useLLImu = false;
    static final boolean useEstimatorOdo = true;
    private final String llNameLeft = Constants2025.LIME_LIGHT_ARPIL_TAG_NAME_LEFT;
    private final String llNameRight = Constants2025.LIME_LIGHT_ARPIL_TAG_NAME_RIGHT;
    private SwerveDrivePoseEstimator est_swerveOdometry;
    private SwerveDriveOdometry swerveOdometry;
    public SwerveModule2025[] mSwerveMods;
    public Pigeon2 gyro;
    private double gyroOffset = 0;
    public Translation2d currentVelTranslation2d = new Translation2d();
    public Timer reset_time = new Timer();

    // below should be copied and modified from example SysIdRoutine
    // SysIdRoutine start
    // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
    MutVoltage m_appliedVoltage = Volts.mutable(0);
    MutDistance m_distance = Meters.mutable(0);
    MutLinearVelocity m_velocity = MetersPerSecond.mutable(0);

    // private final MutableMeasure<Voltage> m_appliedVoltage =
    // mutable(Volts.of(0));
    // Mutable holder for unit-safe linear distance values, persisted to avoid
    // reallocation.
    // private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
    // Mutable holder for unit-safe linear velocity values, persisted to avoid
    // reallocation.
    // private final MutableMeasure<Velocity<Distance>> m_velocity =
    // mutable(MetersPerSecond.of(0));

    private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
            // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                    // Tell SysId how to plumb the driving voltage to the motors.
                    volts -> {
                        for (SwerveModule2025 mod : mSwerveMods) {
                            mod.getmDriveMotor().setVoltage(volts.in(Volts));
                        }
                        // m_leftMotor.setVoltage(volts.in(Volts));
                        // m_rightMotor.setVoltage(volts.in(Volts));
                    },
                    // Tell SysId how to record a frame of data for each motor on the mechanism
                    // being
                    // characterized.
                    log -> {
                        for (SwerveModule2025 mod : mSwerveMods) {
                            // Record a frame for the left motors. Since these share an encoder, we consider
                            // the entire group to be one motor.

                            log.motor(mod.getmDriveMotor().getDescription())
                                    .voltage(
                                            // Volts.mutable(mod.getmDriveMotor().get() *
                                            // RobotController.getBatteryVoltage()))
                                            m_appliedVoltage.mut_replace(
                                                    mod.getmDriveMotor().get() * RobotController.getBatteryVoltage(),
                                                    Volts))
                                    .linearPosition(m_distance.mut_replace((useEstimatorOdo ? est_swerveOdometry.getEstimatedPosition() : swerveOdometry.getPoseMeters())
                                            .getTranslation().getDistance(new Translation2d(0, 0)), Meters))
                                    .linearVelocity(
                                            m_velocity.mut_replace(mod.getState().speedMetersPerSecond,
                                                    MetersPerSecond));
                        }
                        // // Record a frame for the left motors. Since these share an encoder, we
                        // consider
                        // // the entire group to be one motor.
                        // log.motor("drive-left")
                        // .voltage(
                        // m_appliedVoltage.mut_replace(
                        // m_leftMotor.get() * RobotController.getBatteryVoltage(), volts))
                        // .linearPosition(m_distance.mut_replace(m_leftEncoder.getDistance(), Meters))
                        // .linearVelocity(
                        // m_velocity.mut_replace(m_leftEncoder.getRate(), MetersPerSecond));
                        // // Record a frame for the right motors. Since these share an encoder, we
                        // consider
                        // // the entire group to be one motor.
                        // log.motor("drive-right")
                        // .voltage(
                        // m_appliedVoltage.mut_replace(
                        // m_rightMotor.get() * RobotController.getBatteryVoltage(), volts))
                        // .linearPosition(m_distance.mut_replace(m_rightEncoder.getDistance(), Meters))
                        // .linearVelocity(
                        // m_velocity.mut_replace(m_rightEncoder.getRate(), MetersPerSecond));
                    },
                    // Tell SysId to make generated commands require this subsystem, suffix test
                    // state in
                    // WPILog with this subsystem's name ("drive")
                    this));

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        resetModulesToAbsolute();
        return m_sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        resetModulesToAbsolute();
        return m_sysIdRoutine.dynamic(direction);
    }
    // SysIdRoutine end

    public Swerve2025() {
        gyro = new Pigeon2(Constants2025.Swerve.pigeonID, Constants2025.canivore_name);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);

        mSwerveMods = new SwerveModule2025[] {
                new SwerveModule2025(0),
                new SwerveModule2025(1),
                new SwerveModule2025(2),
                new SwerveModule2025(3)
        };
        reset_time.start();
        if (useEstimatorOdo) {
            est_swerveOdometry = new SwerveDrivePoseEstimator(
                Constants2025.Swerve.swerveKinematics, 
                getGyroYaw(), 
                getModulePositions(),
                new Pose2d(),
                VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
                VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));
        }
        else {
            swerveOdometry = new SwerveDriveOdometry(Constants2025.Swerve.swerveKinematics, getGyroYaw(), getModulePositions());
        }
    }

    public void resetGyroOffset(double v) {
        gyroOffset = v;
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop, double maxSpeedScale) {
        // System.out.println("trans x, y: " + translation.getX() + ", " + translation.getY());
        SwerveModuleState[] swerveModuleStates = Constants2025.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation,
                        getHeading())
                        : new ChassisSpeeds(
                                translation.getX(),
                                translation.getY(),
                                rotation));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants2025.Swerve.maxSpeed * maxSpeedScale);

        for (SwerveModule2025 mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop, maxSpeedScale);
        }
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants2025.Swerve.maxSpeed);

        for (SwerveModule2025 mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule2025 mod : mSwerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule2025 mod : mSwerveMods) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public Pose2d getPose() {
        return useEstimatorOdo ? est_swerveOdometry.getEstimatedPosition() : swerveOdometry.getPoseMeters();
    }

    public void setPose(Pose2d pose) {
        if (useEstimatorOdo)
        {
            est_swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
        }
        else {
            swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
        }       
    }

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading) {
        if (useEstimatorOdo) {
            // System.out.println("=========ppppppp ------ > " + heading);
            est_swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(),
            new Pose2d(getPose().getTranslation(), heading));
        }
        else {
            swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(),
            new Pose2d(getPose().getTranslation(), heading));
        }

    }

    public void zeroHeading() {
        // setHeading(new Rotation2d(Units.degreesToRadians(180)));
        setHeading(new Rotation2d());
    }

    public void setHeading(double angle) {
        setHeading(new Rotation2d(Units.degreesToRadians(angle)));
    }

    public Rotation2d getGyroYaw() {
        // return new Rotation2d();
        SmartDashboard.putNumber("gryo111", gyro.getYaw().getValueAsDouble());
        return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
    }

    public void resetModulesToAbsolute() {
        for (SwerveModule2025 mod : mSwerveMods) {
            mod.resetToAbsolute();
        }
    }

    @Override
    public void periodic() {
        if (useLLImu) {
            if (Robot.inst.isDisabled()) {
                LimelightHelpers.SetIMUMode(llNameLeft, 1);
                LimelightHelpers.SetIMUMode(llNameRight, 1);
            }
            else {
                LimelightHelpers.SetIMUMode(llNameLeft, 2);
                LimelightHelpers.SetIMUMode(llNameRight, 2);
            }            
        }
        else {
            LimelightHelpers.SetIMUMode(llNameLeft, 0);
            LimelightHelpers.SetIMUMode(llNameRight, 0);
        }

        if (useEstimatorOdo) {
            est_swerveOdometry.update(getGyroYaw(), getModulePositions());
        }
        else {
            swerveOdometry.update(getGyroYaw(), getModulePositions());
        }
        
        if (this.currentVelTranslation2d.getNorm() < 0.01 && reset_time.hasElapsed(10)) {
            reset_time.reset();
            for (SwerveModule2025 mod : mSwerveMods) {
                mod.resetToAbsolute();
            }
        }
        for (SwerveModule2025 mod : mSwerveMods) {
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
        }
        SmartDashboard.putNumber("Gyro", getGyroYaw().getDegrees());

        if (useEstimatorOdo) {
            // System.out.println("===============>");
            updateOdometryWithVision(llNameLeft);
            updateOdometryWithVision(llNameRight);
        }
    }

    private void updateOdometryWithVision(String llName) {
        boolean useMegaTag2 = true; // set to false to use MegaTag1
        boolean doRejectUpdate = false;

        // int[] validIDs = {17};
        // LimelightHelpers.SetFiducialIDFiltersOverride(llName, validIDs);

        if (useMegaTag2 == false) {
            LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(llName);

            if (mt1.tagCount == 1 && mt1.rawFiducials.length == 1) {
                if (mt1.rawFiducials[0].ambiguity > .7) {
                    doRejectUpdate = true;
                }
                if (mt1.rawFiducials[0].distToCamera > 3) {
                    doRejectUpdate = true;
                }
            }
            if (mt1.tagCount == 0) {
                doRejectUpdate = true;
            }

            if (!doRejectUpdate) {
                est_swerveOdometry.setVisionMeasurementStdDevs(VecBuilder.fill(.5, .5, 9999999));
                est_swerveOdometry.addVisionMeasurement(
                        mt1.pose,
                        mt1.timestampSeconds);
            }
        } else if (useMegaTag2 == true) {
            LimelightHelpers.SetRobotOrientation(llName,
                est_swerveOdometry.getEstimatedPosition().getRotation().getDegrees(),
                // gyro.getYaw().getValueAsDouble() - gyroOffset,
                 0, 0, 0, 0, 0);
            if (StateController.getInstance().useVisionOdometry) {
                LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(llName);
                if (mt2 != null) {
                    Pose2d pos = new Pose2d(mt2.pose.getX(), mt2.pose.getY(), mt2.pose.getRotation());
                    if (Math.abs(gyro.getAngularVelocityZWorld().getValueAsDouble()) > 360) // if our angular velocity is greater than 360 degrees per second,
                                                        // ignore vision updates
                    {
                        doRejectUpdate = true;
                    }
                    if (mt2.tagCount == 0) {
                        doRejectUpdate = true;
                    }
                    
                    if (!doRejectUpdate) {
                        // System.out.println("===============>" + llName + ": " + pos.toString());
                        est_swerveOdometry.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
                        est_swerveOdometry.addVisionMeasurement(
                                pos,
                                mt2.timestampSeconds);
                    }                
                }
            }
        }
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return Constants2025.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds, DriveFeedforwards driveFeedForward) {
        var states = Constants2025.Swerve.swerveKinematics.toSwerveModuleStates(robotRelativeSpeeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants2025.Swerve.maxSpeed);

        setModuleStates(states);
    }

    /**
     * @param pathName
     * @return
     */
    // public Command followPathCommand(String pathName) {

    // PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
    // SmartDashboard.putString("instance1: ", path.toString());
    // // return new PathPlannerAuto("testAuto");
    // // AutoBuilder.configureHolonomic
    // return new FollowPathHolonomic(
    // path,
    // this::getPose, // Robot pose supplier
    // this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT
    // RELATIVE
    // this::driveRobotRelative, // Method that will drive the robot given ROBOT
    // RELATIVE ChassisSpeeds
    // new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should
    // likely live in your
    // // Constants class
    // new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
    // new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
    // 4.5, // Max module speed, in m/s
    // 0.4, // Drive base radius in meters. Distance from robot center to furthest
    // module.
    // new ReplanningConfig() // Default path replanning config. See the API for the
    // options here
    // ),
    // () -> {
    // // Boolean supplier that controls when the path will be mirrored for the red
    // // alliance
    // // This will flip the path being followed to the red side of the field.
    // // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

    // var alliance = DriverStation.getAlliance();
    // if (alliance.isPresent()) {
    // return alliance.get() == DriverStation.Alliance.Red;
    // }
    // return false;
    // },
    // this // Reference to this subsystem to set requirements
    // );
    // }

    public void configPathPlanner() {
        RobotConfig config = null;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }
        // PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
        // SmartDashboard.putString("instance1: ", path.toString());
        if (config != null) {
            AutoBuilder.configure(
                this::getPose,
                this::setPose,
                this::getRobotRelativeSpeeds,
                this::driveRobotRelative,
                new PPHolonomicDriveController( // HolonomicPathFollowerConfig, this should likely live in your
                        new PIDConstants(12, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(6.0, 0.0, 0.0) // Rotation PID constants
                ),
                config,
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    // var alliance = DriverStation.getAlliance();
                    // if (alliance.isPresent()) {
                    //     return alliance.get() == DriverStation.Alliance.Red;
                    // }
                    return false;

                    // return StateController.getInstance().myAlliance ==
                    // DriverStation.Alliance.Red;
                },
                this // Reference to this subsystem to set requirements
            );
        }
    }

    public Command followPathPlannerAuto(String autoName) {
       
        // this.configPathPlanner();
        return new PathPlannerAuto(autoName);
    }
}