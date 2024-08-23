package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.CanID;
// import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants;
import frc.robot.StateController;
import frc.robot.subsystems.Trigger.ShootTrigger;
import frc.robot.subsystems.Trigger.ShootTrigger.TriggerState;

public class Intake extends SubsystemBase {
  private ShootTrigger i_trigger;
  private CANSparkMax intakeLeftMotor = new CANSparkMax(Constants.Intake.intakeLeftMotorID,MotorType.kBrushless);
  private CANSparkMax intakeRightMotor = new CANSparkMax(Constants.Intake.intakeRightMotorID,MotorType.kBrushless);
  private TalonFX intakeFalconMotor = new TalonFX(Constants.Intake.intakeFalconMotorID,Constants.canivore_name);
  REVLibError revError;
  RelativeEncoder m_encoder;
  RelativeEncoder m_intakeEncoder;
  private TalonFXConfiguration intakeFalconMotorConfig = new TalonFXConfiguration();
  private DigitalInput intakeSensor = new DigitalInput(0);
  public boolean isSensored = false;
  private VelocityVoltage intakeVelDutycycle = new VelocityVoltage(0);
  
  // private static Intake instance;
  // public static Intake getInstance(){
  //   if(instance == null){
  //     instance = new Intake();
  //   }
  //   return instance;
  // }

  public Intake(ShootTrigger iTrigger) {
    this.i_trigger = iTrigger;
    intakeLeftMotor.restoreFactoryDefaults();
    intakeRightMotor.restoreFactoryDefaults();
    intakeLeftMotor.setIdleMode(IdleMode.kCoast);
    intakeRightMotor.setIdleMode(IdleMode.kCoast);
    intakeLeftMotor.setInverted(true);
    intakeRightMotor.setInverted(false);
    intakeLeftMotor.setSmartCurrentLimit(30, 80, 5700);
    intakeRightMotor.setSmartCurrentLimit(30, 80, 5700);
    intakeLeftMotor.burnFlash();
    intakeRightMotor.burnFlash();
    for(int i=0;i<5;i++){
      intakeRightMotor.follow(intakeLeftMotor,true);
      if(revError==REVLibError.kOk)break;
    }

    m_intakeEncoder = intakeLeftMotor.getEncoder();
    m_encoder = intakeRightMotor.getEncoder();
    m_intakeEncoder.setPosition(0);

    intakeFalconMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    intakeFalconMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    intakeFalconMotorConfig.MotorOutput.DutyCycleNeutralDeadband = 0.04;

    /* Current Limiting */
    intakeFalconMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    intakeFalconMotorConfig.CurrentLimits.SupplyCurrentLimit = 20;
    intakeFalconMotorConfig.CurrentLimits.SupplyCurrentThreshold = 30;
    intakeFalconMotorConfig.CurrentLimits.SupplyTimeThreshold = 0.02;

     /* Open and Closed Loop Ramping */
    intakeFalconMotorConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 2;
    intakeFalconMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 1;

    intakeFalconMotorConfig.Slot0.kP = Constants.Intake.KP;
    intakeFalconMotorConfig.Slot0.kI = Constants.Intake.KI;
    intakeFalconMotorConfig.Slot0.kD = Constants.Intake.KD;
    intakeFalconMotorConfig.Slot0.kV = Constants.Intake.KV;
    intakeFalconMotorConfig.Slot0.kS = Constants.Intake.KS;
    intakeFalconMotorConfig.Slot0.kA = Constants.Intake.KA;

    intakeFalconMotor.getConfigurator().apply(intakeFalconMotorConfig);

  }

  public enum IntakeState{
    Stop,
    Intake,
    Intake_tiny_reverse,
    Reverse,
    IDLE,
    Intake_sensored,
    NoteReady
  }


  public IntakeState intakeState = IntakeState.Stop;

  public void update(){
    this.update(this.intakeState);
  }

  public void update(IntakeState state){
    this.intakeState = state;
    switch(state){
      case Stop:
        intakeSparkStop();
        intakeFalconStop();
        if(this.i_trigger.triggerState!=TriggerState.Shootout){
          this.i_trigger.update(TriggerState.Stop);
        }
        this.intakeState = IntakeState.IDLE;
      break;
      case IDLE:
      break;
      case NoteReady:
        intakeSparkStop();
        intakeFalconStop();
        this.i_trigger.update(TriggerState.Stop);
      break;
      case Intake_sensored:
        if(!isSensored()){
          this.i_trigger.setNoteReady(true);
          this.intakeState = IntakeState.NoteReady;
          
          intakeSparkStop();
          intakeFalconStop();
          this.i_trigger.update(TriggerState.Stop);
        }
        this.isSensored = false;
      break;
      case Intake:
        if(!this.i_trigger.getNoteReady()){
          intakeSparkControl(Constants.Intake.ForwardSpeed_spark);
          intakeFalconControl(Constants.Intake.ForwardSpeed_falcon);
          this.i_trigger.update(TriggerState.HelpIntake);
          if(isSensored()){
            this.intakeState = IntakeState.Intake_sensored;
            this.isSensored = true;
            StateController.getInstance().intakeAimStop = true;
          }
        }
      break;
      case Reverse:
        if(!this.i_trigger.getNoteReady()){
          intakeSparkControl(Constants.Intake.ReverseSpeed_spark);
          intakeFalconControl(Constants.Intake.ReverseSpeed_falcon);        
          this.i_trigger.update(TriggerState.HelpIntakeReverse);
          if(this.isSensored){
            this.isSensored = false;
          }
        }
      break;
      default:
        break;
    }

  }

  public void intakeSparkControl(double speed){
    intakeLeftMotor.set(speed);
    intakeRightMotor.set(speed);
  }

  public void intakeSparkStop(){
    intakeLeftMotor.stopMotor();
    intakeRightMotor.stopMotor();
  }

  public void intakeFalconControl(double speed){
    intakeVelDutycycle.Velocity = speed;
    intakeFalconMotor.setControl(intakeVelDutycycle);
  }

  public void intakeFalconStop(){
    intakeFalconMotor.stopMotor();
  }
  
  public boolean isSensored(){
    return !intakeSensor.get();
  }

  public boolean isTriggerOk() {
    return i_trigger.getNoteReady();
  }

  @Override
  public void periodic() {    
    // SmartDashboard.putBoolean("intakeSensor0", intakeSensor.get());
    // SmartDashboard.putString("intakeState", this.intakeState.toString());
    // SmartDashboard.putNumber("intake Encoder", m_encoder.getPosition());
  }
}
