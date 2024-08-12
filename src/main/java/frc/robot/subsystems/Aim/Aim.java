package frc.robot.subsystems.Aim;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Aim  extends SubsystemBase {
    public Aim() {
    }

    public void update() {

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
        double kP = .035;

        // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the
        // rightmost edge of
        // your limelight 3 feed, tx should return roughly 31 degrees.
        double targetingAngularVelocity = LimelightHelpers.getTX("limelight") * kP;

        // convert to radians per second for our drive method
        targetingAngularVelocity *= Constants.Swerve.maxAngularVelocity;

        // invert since tx is positive when the target is to the right of the crosshair
        targetingAngularVelocity *= -1.0;

        return targetingAngularVelocity;
    }

    // simple proportional ranging control with Limelight's "ty" value
    // this works best if your Limelight's mount height and target mount height are
    // different.
    // if your limelight and target are mounted at the same or similar heights, use
    // "ta" (area) for target ranging rather than "ty"
    public double limelight_range_proportional() {
        double kP = .1;
        double targetingForwardSpeed = LimelightHelpers.getTY("limelight") * kP;
        targetingForwardSpeed *= Constants.Swerve.maxSpeed;
        targetingForwardSpeed *= -1.0;
        return targetingForwardSpeed;
    }


    @Override
    public void periodic() {

    }


}
