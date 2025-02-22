package frc.robot.subsystems.MoveTo;

import java.util.List;
import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindThenFollowPath;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ControlPadHelper;
import frc.robot.GlobalConfig;
import frc.robot.StateController;
import frc.robot.subsystems.Swerve2025;

public class MoveTo2025 extends SubsystemBase{
    public enum MOVE_TO_CMD_STATE {
        MOVE_TO_CMD_STATE_UNKOWN,
        MOVE_TO_CMD_STATE_RUNNING,
        MOVE_TO_CMD_STATE_FINISHED,
    }
    Swerve2025 s_Swerve;
    Command moveCmd = null;

    private boolean isDidScheduled = false;

    public MoveTo2025(Swerve2025 swerve){
        s_Swerve = swerve;
    }
    
    public void init(Pose2d targetPose) {
        moveCmd = null;
        isDidScheduled = false;

        Pose2d robotPos = s_Swerve.getPose();
        moveCmd = createPathCmd(robotPos, targetPose);
        if (moveCmd != null) {
            moveCmd.schedule();
            
        }
    }

    public void init(String pathname) {
        moveCmd = null;
        isDidScheduled = false;

        moveCmd = createPathCmd(pathname);
        if (moveCmd != null) {
            moveCmd.schedule();
        }
    }

    private Command createPathCmd(String pathName) {
        PathPlannerPath path = GlobalConfig.getAimPath(pathName);
                // Create the constraints to use while pathfinding. The constraints defined in the path will only be used for the path.
        PathConstraints constraints = new PathConstraints(
            3.0, 3.0,
            Units.degreesToRadians(540), Units.degreesToRadians(720));

            System.out.println("---------------> 3");
        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        // return AutoBuilder.pathfindToPose(to, constraints, 0);
        return AutoBuilder.pathfindThenFollowPath(
            path,
            constraints);
    }

    private Command createPathCmd(Pose2d from, Pose2d to) {

        // System.out.println("==============================");
        // System.out.println("==============================");
        // System.out.println("==============================");
        // System.out.println("from: " + from.toString());
        // System.out.println("to: " + to.toString());

        ControlPadHelper.ControlPadInfo.ControlPadInfoData info = ControlPadHelper.getControlPadInfo();
        if (info == null) {
            return null;
        }
        int fid = (int)info.aprilTagId;
        String leftORright = info.branch == -1 ? "left" : "right";
        String pathName = String.format("ap%d_%s", fid, leftORright);
        PathPlannerPath path = GlobalConfig.getAimPath(pathName);
        System.out.println("---------------> 2");
        // Create the constraints to use while pathfinding. The constraints defined in the path will only be used for the path.
        PathConstraints constraints = new PathConstraints(
                3.0, 3.0,
                Units.degreesToRadians(540), Units.degreesToRadians(720));

                System.out.println("---------------> 3");
        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        // return AutoBuilder.pathfindToPose(to, constraints, 0);
        return AutoBuilder.pathfindThenFollowPath(
            path,
            constraints);

        // List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(from, to);
        // PathConstraints constraints = new PathConstraints(
        //     1.0, 2.0,
        //     Units.degreesToRadians(540), Units.degreesToRadians(720));

        // PathPlannerPath path = new PathPlannerPath(waypoints, constraints, new IdealStartingState(0.05, from.getRotation()), new GoalEndState(0.05, to.getRotation()));
        // path.preventFlipping = true;
    
        // return AutoBuilder.followPath(path);
    }

    public MOVE_TO_CMD_STATE getAimMoveCmdState() {

        if (moveCmd.isScheduled()) {
            isDidScheduled = true;
            return MOVE_TO_CMD_STATE.MOVE_TO_CMD_STATE_RUNNING;
        }

        if (isDidScheduled && (moveCmd.isFinished() || !moveCmd.isScheduled())) {
            return MOVE_TO_CMD_STATE.MOVE_TO_CMD_STATE_FINISHED;
        }

        return MOVE_TO_CMD_STATE.MOVE_TO_CMD_STATE_UNKOWN;
    }

    public void cancelAimMoveCmd() {
        if (moveCmd != null) {
            moveCmd.cancel();
            moveCmd = null;
            StateController.getInstance().aimMoveCmdRunning = false;
        }
    }

    @Override
    public void periodic() {

        if (this.moveCmd != null) {
            MOVE_TO_CMD_STATE state = getAimMoveCmdState();
            SmartDashboard.putString("Move to ...", "state: " + state.name());    
            SmartDashboard.putString("Move to ...", "isFinished: " + moveCmd.isFinished() + " , isSchudled: " + moveCmd.isScheduled());        
        }

    }
}
