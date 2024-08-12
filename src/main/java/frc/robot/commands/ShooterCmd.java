package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.Shooter.ShooterState;

public class ShooterCmd extends Command {
  private Shooter m_Shooter;
  // private Timer ellapsedTime_rightTrigger = new Timer();

  public ShooterCmd(Shooter m_Shooter, ShooterState ss) {
    this.m_Shooter = m_Shooter;
    this.m_Shooter.shooterState = ss;
    addRequirements(m_Shooter);
    schedule();
  }

  @Override
  public void initialize() {
    // ellapsedTime_rightTrigger.start();
  }


  @Override
  public void execute() {  
    m_Shooter.update();
  }


  @Override
  public void end(boolean interrupted) {
  }


  @Override
  public boolean isFinished() {
    return false;
  }
}

