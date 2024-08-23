package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.StateController;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Candle.Candle;
import frc.robot.subsystems.IntakeAim.IntakeAim;

public class ResetCmd extends Command {
    Swerve m_Swerve;
    private IntakeAim m_IntakeAim;
    public ResetCmd(Swerve swerve, IntakeAim intakeAim) {
        m_Swerve = swerve;
        m_IntakeAim = intakeAim;
        schedule();
    }

    @Override
    public void initialize() {
        StateController sc = StateController.getInstance();
        sc.isAutoAimming = false;
        sc.isAutoIntakeAimming = false;
        m_IntakeAim.reset();
        m_Swerve.drive(
            new Translation2d(0, 0),
            0,
            false,
            true);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
