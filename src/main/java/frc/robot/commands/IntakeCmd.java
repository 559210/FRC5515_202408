package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeState;

public class IntakeCmd extends Command {
  private Intake m_Intake;
  private DoubleSupplier leftTriggerButton;
  private DoubleSupplier rightTriggerButton;
  // private Timer ellapsedTime_Trigger = new Timer();

  public IntakeCmd(Intake m_IntakeSubsystem,DoubleSupplier LTButton,DoubleSupplier RTButton) {
    this.m_Intake = m_IntakeSubsystem;
    this.leftTriggerButton = LTButton;
    this.rightTriggerButton = RTButton;
    addRequirements(m_IntakeSubsystem);
    schedule();
  }


  @Override
  public void initialize() {
    // ellapsedTime_Trigger.start();
  }


  @Override
  public void execute() {
    // // SmartDashboard.putNumber("ellapsedTime_leftTrigger",ellapsedTime_Trigger.get());
    // if(ellapsedTime_Trigger.hasElapsed(0.5)){
    //   ellapsedTime_Trigger.reset();
      if(leftTriggerButton.getAsDouble()>0.15){
        m_Intake.update(IntakeState.Reverse);
      }
      if(rightTriggerButton.getAsDouble()>0.15){
        if(m_Intake.intakeState!=IntakeState.NoteReady&&m_Intake.intakeState!=IntakeState.Intake_sensored)
          m_Intake.update(IntakeState.Intake);
      }
      if(rightTriggerButton.getAsDouble()<=0.15&&leftTriggerButton.getAsDouble()<=0.15){
        m_Intake.update(IntakeState.Stop);
      }
    // }else{
      
    // }
    m_Intake.update();
  }


  @Override
  public void end(boolean interrupted) {
  }


  @Override
  public boolean isFinished() {
    return false;
  }
}

