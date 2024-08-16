package frc.robot.subsystems.IntakeAim;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.StateController;
import frc.robot.subsystems.Swerve;
import frc.robot.utils.CoordinaryFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class IntakeAim  extends SubsystemBase {
    PIDController pidRot = new PIDController(0.015, 0.01, 0.00125);
    PIDController pidTrans = new PIDController(0.3, 0, 0);
    CoordinaryFilter filterX;
    CoordinaryFilter filterY;
    public IntakeAim() {
    }

    public void update() {

    }

    public void reset() {
        pidRot = new PIDController(0.015, 0.01, 0.00125);
        pidTrans = new PIDController(0.3, 0, 0);
        filterX = null;
        filterY = null;
    }

    protected void lazyCreateFilter() {
        if (filterX == null) {
            filterX = new CoordinaryFilter(0.6);
        }
        if (filterY == null) {
            filterY = new CoordinaryFilter(0.6);
        }
    }

    public boolean isTargetValid() {
        return LimelightHelpers.getTV(Constants.LIME_LIGHT_NOTE_NAME);
    }

    // simple proportional turning control with Limelight.
    // "proportional control" is a control algorithm in which the output is
    // proportional to the error.
    // in this case, we are going to return an angular velocity that is proportional
    // to the
    // "tx" value from the Limelight.
    public double limelight_aim_proportional() {
        lazyCreateFilter();
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
        double tx = LimelightHelpers.getTX(Constants.LIME_LIGHT_NOTE_NAME);
        
        SmartDashboard.putNumber("o tx", tx);
        StateController.getInstance().intakeAimTx = tx;
        tx = filterX.calc(tx);
        SmartDashboard.putNumber("n tx", tx);
        StateController.getInstance().aimTx = tx;
        // double targetingAngularVelocity = LimelightHelpers.getTX(Constants.LIME_LIGHT_AIM_NAME) * kP;

        double targetingAngularVelocity = pidRot.calculate(tx);
        // convert to radians per second for our drive method
        targetingAngularVelocity *= Constants.Swerve.maxAngularVelocity;

        // pidRot.calculate(-LimelightHelpers.getTX(Constants.LIME_LIGHT_AIM_NAME));

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
        lazyCreateFilter();
        // double kP = .1;
        // double targetingForwardSpeed = LimelightHelpers.getTY(Constants.LIME_LIGHT_AIM_NAME) * kP;
        double ty = LimelightHelpers.getTY(Constants.LIME_LIGHT_NOTE_NAME);
        StateController.getInstance().intakeAimTy = ty;
        ty = filterY.calc(ty);
        StateController.getInstance().aimTy = ty;
        double targetingForwardSpeed = pidTrans.calculate(ty);
        targetingForwardSpeed *= Constants.Swerve.maxSpeed;
        targetingForwardSpeed *= -1.0;

        // return 0;
        return targetingForwardSpeed;
    }


    @Override
    public void periodic() {

    }
}
