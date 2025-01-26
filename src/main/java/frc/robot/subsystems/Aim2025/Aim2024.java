package frc.robot.subsystems.Aim2025;

import java.util.List;
import java.util.stream.IntStream;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.GlobalConfig;
import frc.robot.LimelightHelpers;
import frc.robot.StateController;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.subsystems.Swerve;
import frc.robot.utils.MiscUtils;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;



public class Aim2024  extends SubsystemBase {
    public enum AIM_MOVE_CMD_STATE {
        AIM_MOVE_CMD_STATE_UNKOWN,
        AIM_MOVE_CMD_STATE_IDLE,
        AIM_MOVE_CMD_STATE_RUNNING,
        AIM_MOVE_CMD_STATE_FINISHED,
        AIM_MOVE_CMD_STATE_CANCELED,
    }

    PIDController pidRot = new PIDController(0.015, 0.01, 0.0015);
    PIDController pidTrans = new PIDController(0.06, 0, .01);

    String llName = "limelight-one";// Constants.LIME_LIGHT_AIM_NAME;

    Swerve s_Swerve;
    Command aimMoveCmd = null;

    float waitForSeeAprilTagTime = 5; // seconds
    boolean isTimeoutCancel = false;   // if not see any aprilTag in a while, cancel the aimMoveCmd
    private boolean isDidScheduled = false;

    public Aim2024(Swerve swerve) {
        s_Swerve = swerve;
        // StateController sc = StateController.getInstance();
        // var array1 = Constants.AprilTag.speakerIds[sc.myAllianceIndex];
        // var array2 = Constants.AprilTag.ampIds[sc.myAllianceIndex];
        // int[] mergedArray = IntStream.concat(IntStream.of(array1), IntStream.of(array2)).toArray();
        // SmartDashboard.putNumber("filters override count", mergedArray.length);
        // SmartDashboard.putNumber("filters override item1", mergedArray[0]);
        // SmartDashboard.putNumber("filters override item1", mergedArray[1]);
        // LimelightHelpers.SetFiducialIDFiltersOverride(llName, mergedArray);
    }

    public boolean test = false;

    public void init() {
        aimMoveCmd = null;
        waitForSeeAprilTagTime = 5;
        isTimeoutCancel = false;
        isDidScheduled = false;
    }
    public void update() {
        if (aimMoveCmd != null) {
            return;
        }
        if (isTargetValid()) {
            Pose2d robotPos = s_Swerve.getPose();

            try {
                aimMoveCmd = createPathCmd(robotPos, new Pose2d(3.56,2.5, Rotation2d.fromDegrees(60)));
                aimMoveCmd.schedule();
                StateController.getInstance().aimMoveCmdRunning = true;                    
            }
            catch (Exception e) {
                System.err.println(e.toString());
            }
        }
        else {
            // SmartDashboard.putBoolean("========>1111", false);
            waitForSeeAprilTagTime -= 0.02; // 50Hz，a frame every 0.02s
            if (waitForSeeAprilTagTime <= 0) {
                isTimeoutCancel = true;
            }
        }
    }

    private Command createPathCmd(Pose2d from, Pose2d to) {

        System.out.println("==============================");
        System.out.println("==============================");
        System.out.println("==============================");
        System.out.println("from: " + from.toString());
        System.out.println("to: " + to.toString());
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(from, to);
        PathConstraints constraints = new PathConstraints(
            2.0, 2.0,
            Units.degreesToRadians(540), Units.degreesToRadians(720));

        PathPlannerPath path = new PathPlannerPath(waypoints, constraints, new IdealStartingState(0.05, Rotation2d.fromDegrees(60)), new GoalEndState(0.05, Rotation2d.fromDegrees(60)));
        path.preventFlipping = true;
        return AutoBuilder.followPath(path);

        // return null;
    }

    // public void update() {
    //     // LimelightResults res = LimelightHelpers.getLatestResults(llName);
    //     if (aimMoveCmd != null) {
    //         return;
    //     }
    //     else {
    //         System.out.println("waitForSeeAprilTagTime: " + waitForSeeAprilTagTime);
    //         // double[] pos = LimelightHelpers.getBotPose_TargetSpace(llName);
    //         // // pos[4] == 0 means ok.
    //         // // pos[4] < 0 robot should turn right, > 0 turn left
    //         if (isTargetValid()) {
    //         // if (!MiscUtils.isAllZero(pos)) {
    //             // Pose3d p3d = LimelightHelpers.toPose3D(pos);
    //             // SmartDashboard.putBoolean("========>1111", true);
    //             // SmartDashboard.putNumberArray("========>1", pos);
    //             // SmartDashboard.putString("--------->2", p3d.toString());
    
    //             // Load the path we want to pathfind to and follow
    //             try {
    //                 // double _fid = LimelightHelpers.getFiducialID(llName);
    //                 // // SmartDashboard.putNumber("fid value", fid);
    //                 // System.out.println("---------------> 1");
                    
    //                 // int fid = (int)Math.round(_fid);
    //                 int fid = 17;
    //                 String pathName = String.format("ap%d_right", fid);
    //                 PathPlannerPath path = GlobalConfig.getAimPath(pathName);
    //                 System.out.println("---------------> 2");
    //                 // Create the constraints to use while pathfinding. The constraints defined in the path will only be used for the path.
    //                 PathConstraints constraints = new PathConstraints(
    //                         3.0, 3.0,
    //                         Units.degreesToRadians(540), Units.degreesToRadians(720));
        
    //                         System.out.println("---------------> 3");
    //                 // Since AutoBuilder is configured, we can use it to build pathfinding commands
    //                 aimMoveCmd = AutoBuilder.pathfindThenFollowPath(
    //                     path,
    //                     constraints);
    //                 System.out.println("---------------> 4");
    //                 // StateController.getInstance().useVisionOdometry = false;
    //                 aimMoveCmd.schedule();
    //                 isDidScheduled = false;
    //                 System.out.println("---------------> 5");
    //                 StateController.getInstance().aimMoveCmdRunning = true;
    //             }
    //             catch (Exception e) {
    //                 e.printStackTrace();
    //             }
    //         }
    //         else {
    //             // SmartDashboard.putBoolean("========>1111", false);
    //             waitForSeeAprilTagTime -= 0.02; // 50Hz，a frame every 0.02s
    //             if (waitForSeeAprilTagTime <= 0) {
    //                 isTimeoutCancel = true;
    //             }
    //         }
    //     } 
    // }

    
    public AIM_MOVE_CMD_STATE getAimMoveCmdState() {
        if (isTimeoutCancel) {
            return AIM_MOVE_CMD_STATE.AIM_MOVE_CMD_STATE_CANCELED;
        }
        if (aimMoveCmd == null) {
            return AIM_MOVE_CMD_STATE.AIM_MOVE_CMD_STATE_IDLE;
        }
        if (aimMoveCmd.isScheduled()) {
            isDidScheduled = true;

            return AIM_MOVE_CMD_STATE.AIM_MOVE_CMD_STATE_RUNNING;
        }

        if (isDidScheduled && aimMoveCmd.isFinished()) {
            return AIM_MOVE_CMD_STATE.AIM_MOVE_CMD_STATE_FINISHED;
        }

        return AIM_MOVE_CMD_STATE.AIM_MOVE_CMD_STATE_UNKOWN;
    }

    public void cancelAimMoveCmd() {
        if (aimMoveCmd != null) {
            aimMoveCmd.cancel();
            aimMoveCmd = null;
            StateController.getInstance().aimMoveCmdRunning = false;
        }
        // StateController.getInstance().useVisionOdometry = true;
    }

    public boolean isTargetValid() {
        // return LimelightHelpers.getTV(llName);
        return true;
    }

    private boolean isSpeaker(int id) {
        SmartDashboard.putNumber("isSpeaker id", id);

        SmartDashboard.putString("isSpeaker true", "unknow");
        SmartDashboard.putString("isSpeaker false", "unknow");
        StateController sc = StateController.getInstance();
        int[] array1 = Constants.AprilTag.speakerIds[sc.myAllianceIndex];
        SmartDashboard.putNumber("array len", array1.length);
        SmartDashboard.putNumber("array item 0", array1[0]);
        for (int element : array1) {
            if (element == id) {
                SmartDashboard.putString("isSpeaker true", "true");
                return true;
            }
        }
        SmartDashboard.putString("isSpeaker false", "false");
        return false;
    }
    // simple proportional turning control with Limelight.
    // "proportional control" is a control algorithm in which the output is
    // proportional to the error.
    // in this case, we are going to return an angular velocity that is proportional
    // to the
    // "tx" value from the Limelight.
    public double limelight_aim_proportional() {
        // kP (constant of proportionality)
        // this is a hand-tuned number that determines the aggressiveness of our
        // proportional control loop
        // if it is too high, the robot will oscillate.
        // if it is too low, the robot will never reach its target
        // if the robot never turns in the correct direction, kP should be inverted.
        // double kP = .015;
        

        // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the
        // rightmost edge of
        // your limelight 3 feed, tx should return roughly 31 degrees.

        double tx = LimelightHelpers.getTX(llName);
        StateController.getInstance().aimTx = tx;
        // double targetingAngularVelocity = LimelightHelpers.getTX(llName) * kP;

        double targetingAngularVelocity = pidRot.calculate(-tx);
        // convert to radians per second for our drive method
        targetingAngularVelocity *= Constants.Swerve.maxAngularVelocity;

        // pidRot.calculate(-LimelightHelpers.getTX(llName));

        // invert since tx is positive when the target is to the right of the crosshair
        targetingAngularVelocity *= -1.0;

        return targetingAngularVelocity;
        // return 0;
    }

    // simple proportional ranging control with Limelight's "ty" value
    // this works best if your Limelight's mount height and target mount height are
    // different.
    // if your limelight and target are mounted at the same or similar heights, use
    // "ta" (area) for target ranging rather than "ty"
    public double limelight_range_proportional() {
        double offsetY = 0;
        double fid = LimelightHelpers.getFiducialID(llName);

        SmartDashboard.putNumber("fid value", fid);
        if (!isSpeaker((int)Math.round(fid))) {
            offsetY = Constants.AprilTag.ampOffsetYInLimeLight;
        }

        
        double ty = LimelightHelpers.getTY(llName) + offsetY;
        StateController.getInstance().aimTy = ty;
        double targetingForwardSpeed = pidTrans.calculate(ty);
        targetingForwardSpeed *= Constants.Swerve.maxSpeed;;
        targetingForwardSpeed *= -1.0;

        // return 0;
        return targetingForwardSpeed;
    }


    @Override
    public void periodic() {

    }
}
