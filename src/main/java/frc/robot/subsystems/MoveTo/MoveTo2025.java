package frc.robot.subsystems.MoveTo;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
        moveCmd = createPathCmd(robotPos, new Pose2d(3.56,2.5, Rotation2d.fromDegrees(60)));
        moveCmd.schedule();
    }

    private Command createPathCmd(Pose2d from, Pose2d to) {

        // System.out.println("==============================");
        // System.out.println("==============================");
        // System.out.println("==============================");
        // System.out.println("from: " + from.toString());
        // System.out.println("to: " + to.toString());
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(from, to);
        PathConstraints constraints = new PathConstraints(
            2.0, 2.0,
            Units.degreesToRadians(540), Units.degreesToRadians(720));

        PathPlannerPath path = new PathPlannerPath(waypoints, constraints, new IdealStartingState(0.05, from.getRotation()), new GoalEndState(0.05, to.getRotation()));
        path.preventFlipping = true;
        return AutoBuilder.followPath(path);
    }

    public MOVE_TO_CMD_STATE getAimMoveCmdState() {

        if (moveCmd.isScheduled()) {
            isDidScheduled = true;
            return MOVE_TO_CMD_STATE.MOVE_TO_CMD_STATE_RUNNING;
        }

        if (isDidScheduled && moveCmd.isFinished()) {
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
        // StateController.getInstance().useVisionOdometry = true;
    }

    @Override
    public void periodic() {

    }
}
