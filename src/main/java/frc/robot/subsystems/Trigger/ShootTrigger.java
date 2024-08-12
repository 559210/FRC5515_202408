
package frc.robot.subsystems.Trigger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
// import frc.robot.Constants.CanID;
// import frc.robot.subsystems.LEDSubsystem.LEDState;


public class ShootTrigger extends SubsystemBase {
  // LEDSubsystem m_LedSubsystem;
  private TalonFX triggerMotor = new TalonFX(Constants.Trigger.triggerMotorID);

  private TalonFXConfiguration triggerMotorConfig = new TalonFXConfiguration();

  private VelocityVoltage triggerDutyCycle = new VelocityVoltage(0);
  private boolean isNoteReady = false;
  public double desireVelocity = 0;
  private Timer ellapsedTime_Reset = new Timer();
  private Timer ellapsedTime_Reset_trigger = new Timer();
  
  // private static ShootingSubsystem instance;
  // public static ShootingSubsystem getInstance(){
  //   if(instance == null){
  //     instance = new ShootingSubsystem();
  //   }
  //   return instance;
  // }


  public ShootTrigger() {
    /* Motor Inverts and Neutral Mode */
    triggerMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    triggerMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    triggerMotorConfig.MotorOutput.DutyCycleNeutralDeadband = 0.04;

    /* Current Limiting */
    triggerMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    triggerMotorConfig.CurrentLimits.SupplyCurrentLimit = 30;
    triggerMotorConfig.CurrentLimits.SupplyCurrentThreshold = 40;
    triggerMotorConfig.CurrentLimits.SupplyTimeThreshold = 0.02;

     /* Open and Closed Loop Ramping */
    triggerMotorConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 2;
    triggerMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 1;

    triggerMotorConfig.Slot0.kP = Constants.Trigger.KP;
    triggerMotorConfig.Slot0.kI = Constants.Trigger.KI;
    triggerMotorConfig.Slot0.kD = Constants.Trigger.KD;
    triggerMotorConfig.Slot0.kV = Constants.Trigger.KV;
    triggerMotorConfig.Slot0.kS = Constants.Trigger.KS;
    triggerMotorConfig.Slot0.kA = Constants.Trigger.KA;
    triggerMotor.getConfigurator().apply(triggerMotorConfig);

    ellapsedTime_Reset.start();
    ellapsedTime_Reset_trigger.start();
  }

  public enum TriggerState{
    Stop,
    IDLE,
    HelpIntake,
    HelpIntakeReverse,
    Shootout
  }

  public TriggerState triggerState = TriggerState.Stop;

  public void update(){
    this.update(this.triggerState);
  }

  public void update(TriggerState state){
    this.triggerState = state;
    switch(state){
      case Stop:
        triggerMotor.stopMotor();
        this.triggerState = TriggerState.IDLE;
      break;
      case IDLE:
      break;
      case HelpIntake:
        triggerMotorControl(Constants.Trigger.HelpIntake);
      break;
      case HelpIntakeReverse:
        triggerMotorControl(Constants.Trigger.HelpIntakeReverse);
      break;
      case Shootout:
        triggerMotorControl(Constants.Trigger.shootout);
      break;
      default:break;
    }
  }


  public void triggerMotorControl(double velocity){
    triggerDutyCycle.Velocity = velocity;
    triggerMotor.setControl(triggerDutyCycle);
  }


  public double getTriggerVelocity(){
    return triggerMotor.getVelocity().getValue();
  }

  public void resetAngleEncoder(){
    if(ellapsedTime_Reset.get()> 3){
      triggerMotor.setPosition(0);
      ellapsedTime_Reset.reset();
    }
  }

  public void setNoteReady(boolean inr){
    this.isNoteReady = inr;
  }

  public boolean getNoteReady(){
    return this.isNoteReady;
  }

  @Override
  public void periodic() {
    // SmartDashboard.putBoolean("shooterSensor0", triggerSensorLeft.get());
    SmartDashboard.putNumber("triggerVelocity", getTriggerVelocity());
    SmartDashboard.putNumber("triggerMotor", triggerMotor.getPosition().getValue());
    SmartDashboard.putString("triggerState", this.triggerState.toString());
    SmartDashboard.putBoolean("isNoteReady", this.isNoteReady);

  }
}
