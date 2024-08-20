package frc.robot.subsystems.Aim;

import java.util.stream.IntStream;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.StateController;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Aim  extends SubsystemBase {
    PIDController pidRot = new PIDController(0.015, 0.01, 0.0015);
    PIDController pidTrans = new PIDController(0.06, 0, .01);

    String llName = Constants.LIME_LIGHT_AIM_NAME;

    public Aim() {
        // StateController sc = StateController.getInstance();
        // var array1 = Constants.AprilTag.speakerIds[sc.myAllianceIndex];
        // var array2 = Constants.AprilTag.ampIds[sc.myAllianceIndex];
        // int[] mergedArray = IntStream.concat(IntStream.of(array1), IntStream.of(array2)).toArray();
        // SmartDashboard.putNumber("filters override count", mergedArray.length);
        // SmartDashboard.putNumber("filters override item1", mergedArray[0]);
        // SmartDashboard.putNumber("filters override item1", mergedArray[1]);
        // LimelightHelpers.SetFiducialIDFiltersOverride(llName, mergedArray);
    }

    public void update() {

    }
    public boolean isTargetValid() {
        return LimelightHelpers.getTV(llName);
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
