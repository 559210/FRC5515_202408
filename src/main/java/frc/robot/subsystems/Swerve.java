package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants;
import frc.robot.StateController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;


import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.*;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;
public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;
    public Translation2d currentVelTranslation2d = new Translation2d();
    public Timer reset_time = new Timer();

    //below should be copied and modified from example SysIdRoutine
    //SysIdRoutine start
    // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
    private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
    // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
    private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
    // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
    private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));

    private final SysIdRoutine m_sysIdRoutine =
        new SysIdRoutine(
            // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                // Tell SysId how to plumb the driving voltage to the motors.
                (Measure<Voltage> volts) -> {
                    for(SwerveModule mod : mSwerveMods){
                        mod.getmDriveMotor().setVoltage(volts.in(Volts));
                    }
                    // m_leftMotor.setVoltage(volts.in(Volts));
                    // m_rightMotor.setVoltage(volts.in(Volts));
                },
                // Tell SysId how to record a frame of data for each motor on the mechanism being
                // characterized.
                log -> {
                    for(SwerveModule mod : mSwerveMods){
                        // Record a frame for the left motors.  Since these share an encoder, we consider
                        // the entire group to be one motor.
                        log.motor(mod.getmDriveMotor().getDescription())
                            .voltage(
                                m_appliedVoltage.mut_replace(
                                    mod.getmDriveMotor().get() * RobotController.getBatteryVoltage(), Volts))
                            .linearPosition(m_distance.mut_replace(swerveOdometry.getPoseMeters().getTranslation().getDistance(new Translation2d(0,0)), Meters))
                            .linearVelocity(
                                m_velocity.mut_replace(mod.getState().speedMetersPerSecond, MetersPerSecond));
                    }       
                    // // Record a frame for the left motors.  Since these share an encoder, we consider
                    // // the entire group to be one motor.
                    // log.motor("drive-left")
                    //     .voltage(
                    //         m_appliedVoltage.mut_replace(
                    //             m_leftMotor.get() * RobotController.getBatteryVoltage(), volts))
                    //     .linearPosition(m_distance.mut_replace(m_leftEncoder.getDistance(), Meters))
                    //     .linearVelocity(
                    //         m_velocity.mut_replace(m_leftEncoder.getRate(), MetersPerSecond));
                    // // Record a frame for the right motors.  Since these share an encoder, we consider
                    // // the entire group to be one motor.
                    // log.motor("drive-right")
                    //     .voltage(
                    //         m_appliedVoltage.mut_replace(
                    //             m_rightMotor.get() * RobotController.getBatteryVoltage(), volts))
                    //     .linearPosition(m_distance.mut_replace(m_rightEncoder.getDistance(), Meters))
                    //     .linearVelocity(
                    //         m_velocity.mut_replace(m_rightEncoder.getRate(), MetersPerSecond));
                },
                // Tell SysId to make generated commands require this subsystem, suffix test state in
                // WPILog with this subsystem's name ("drive")
                this));
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.dynamic(direction);
    }
    //SysIdRoutine end


    public Swerve() {
        gyro = new Pigeon2(Constants.Swerve.pigeonID,Constants.canivore_name);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants,0),
            new SwerveModule(1, Constants.Swerve.Mod1.constants,1),
            new SwerveModule(2, Constants.Swerve.Mod2.constants,1),
            new SwerveModule(3, Constants.Swerve.Mod3.constants,0)
        };
        reset_time.start();
        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getGyroYaw(), getModulePositions());
    }

    int drivecount = 0;
    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        drivecount++;
        SmartDashboard.putNumber("drive number", drivecount);
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getHeading()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    int poseCount = 0;
    public void setPose(Pose2d pose) {
        poseCount++;
        SmartDashboard.putNumber("setPose count", poseCount);
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public Rotation2d getHeading(){
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading){
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading(){
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    public Rotation2d getGyroYaw() {
        // return new Rotation2d();
        SmartDashboard.putNumber("gryo111", gyro.getYaw().getValue());
        return Rotation2d.fromDegrees(gyro.getYaw().getValue());
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    @Override
    public void periodic(){
        swerveOdometry.update(getGyroYaw(), getModulePositions());
        if(this.currentVelTranslation2d.getNorm()<0.01 && reset_time.hasElapsed(10)){
            reset_time.reset();
            for(SwerveModule mod:mSwerveMods){
                mod.resetToAbsolute();
            }
        }
        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }
        SmartDashboard.putNumber("Gyro",getGyroYaw().getDegrees());
    }

    public ChassisSpeeds getRobotRelativeSpeeds(){
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
    }
    
    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds){
        var states = Constants.Swerve.swerveKinematics.toSwerveModuleStates(robotRelativeSpeeds);
    
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Swerve.maxSpeed);
    
        setModuleStates(states);
    }

    /**
     * @param pathName
     * @return
     */
    public Command followPathCommand(String pathName) {
        
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
        SmartDashboard.putString("instance1: ", path.toString());
        // return new PathPlannerAuto("testAuto");
        // AutoBuilder.configureHolonomic
        return new FollowPathHolonomic(
                path,
                this::getPose, // Robot pose supplier
                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your
                                                 // Constants class
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                        4.5, // Max module speed, in m/s
                        0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );
    }

    public Command followPathPlannerAuto(String autoName) {
        // PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
        // SmartDashboard.putString("instance1: ", path.toString());
        AutoBuilder.configureHolonomic(
            this::getPose,
            this::setPose,
            this::getRobotRelativeSpeeds,
            this::driveRobotRelative,
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your
                                                // Constants class
                    new PIDConstants(1.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(1.0, 0.0, 0.0), // Rotation PID constants
                    3, // Max module speed, in m/s
                    0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig(true, true) // Default path replanning config. See the API for the options here
            ),
            () -> {
                // Boolean supplier that controls when the path will be mirrored for the red
                // alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                // var alliance = DriverStation.getAlliance();
                // if (alliance.isPresent()) {
                //     return alliance.get() == DriverStation.Alliance.Red;
                // }
                // return false;

                return StateController.getInstance().myAlliance == DriverStation.Alliance.Red;
            },
            this // Reference to this subsystem to set requirements
        );
        return new PathPlannerAuto(autoName);
        // 
    }
}