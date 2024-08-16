package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.StateController;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;


public class TeleopSwerve extends Command {    
    private Swerve s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;

    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
    }
    
    int cout = 0;
    @Override
    public void execute() {
        cout++;
        SmartDashboard.putNumber("tele exe", cout);
        StateController sc = StateController.getInstance();
        SmartDashboard.putBoolean("autoAiming", sc.isAutoAimming);
        if (sc.isAutoAimming || sc.isAutoIntakeAimming) {
            // i don't know if this excute function is running when other command executing. by majun
            
        }
        else {
            /* Get Values, Deadband*/
            double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
            double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
            double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);

            /* Drive */
            s_Swerve.drive(
                new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
                rotationVal * Constants.Swerve.maxAngularVelocity, 
                !robotCentricSup.getAsBoolean(), 
                // true,
                true
            );
        }

    }
}