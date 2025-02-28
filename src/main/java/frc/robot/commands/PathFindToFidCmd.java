
package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.StateController;
import frc.robot.subsystems.Swerve2025;
import frc.robot.subsystems.Aim2025.Aim2025;



public class PathFindToFidCmd extends Command {
    public enum AIM_MOVE_CMD_STATE {
        AIM_MOVE_CMD_STATE_UNKOWN,
        AIM_MOVE_CMD_STATE_IDLE,
        AIM_MOVE_CMD_STATE_RUNNING,
        AIM_MOVE_CMD_STATE_FINISHED,
        AIM_MOVE_CMD_STATE_CANCELED,
    }

    String pathName;
    Command aimMoveCmd = null;

    public PathFindToFidCmd(String pn) {
        pathName = pn;
        schedule();
    }

    @Override
    public void initialize() {
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
            path.preventFlipping = true;
            System.out.println("path find 000000000");
            // Create the constraints to use while pathfinding. The constraints defined in
            // the path will only be used for the path.
            PathConstraints constraints = new PathConstraints(
                    3.0, 3.0,
                    Units.degreesToRadians(540), Units.degreesToRadians(720));
    
            // Since AutoBuilder is configured, we can use it to build pathfinding commands
            System.out.println("path find 111111111111");
            aimMoveCmd = AutoBuilder.pathfindThenFollowPath(
                    path,
                    constraints);
            System.out.println("path find 222222222222");
            aimMoveCmd.schedule();
            System.out.println("path find 333333333333333");
            StateController.getInstance().aimMoveCmdRunning = true;
        }
        catch (Exception e) {
            e.printStackTrace();
        }
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        if (aimMoveCmd != null) {
            aimMoveCmd.cancel();
            aimMoveCmd = null;
            StateController.getInstance().aimMoveCmdRunning = false;
        }
    }

    // public AIM_MOVE_CMD_STATE getState() {
    //     if (aimMoveCmd == null) {
    //         return AIM_MOVE_CMD_STATE.AIM_MOVE_CMD_STATE_IDLE;
    //     }
    //     if (aimMoveCmd.isScheduled()) {
    //         return AIM_MOVE_CMD_STATE.AIM_MOVE_CMD_STATE_RUNNING;
    //     }

    //     if (aimMoveCmd.isFinished()) {
    //         return AIM_MOVE_CMD_STATE.AIM_MOVE_CMD_STATE_FINISHED;
    //     }

    //     return AIM_MOVE_CMD_STATE.AIM_MOVE_CMD_STATE_UNKOWN;
    // }
    @Override
    public boolean isFinished() {
        if (aimMoveCmd == null) {
            return true;
        }
        return aimMoveCmd.isFinished();
    }
}
