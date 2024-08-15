
package frc.robot.subsystems.Shooter;

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
import frc.robot.subsystems.Trigger.ShootTrigger;
import frc.robot.subsystems.Trigger.ShootTrigger.TriggerState;


public class Shooter extends SubsystemBase {
  // LEDSubsystem m_LedSubsystem;
  private ShootTrigger s_trigger;
  private TalonFX flywheelUpMotor = new TalonFX(Constants.Shooter.flywheelUpMotorID);
  private TalonFX flywheelDownMotor = new TalonFX(Constants.Shooter.flywheelDownMotorID);

  private TalonFXConfiguration flywheelUpconfig = new TalonFXConfiguration();
  private TalonFXConfiguration flywheelDownconfig = new TalonFXConfiguration();

  private VelocityVoltage flywheelUpVelDutycycle = new VelocityVoltage(0);
  private VelocityVoltage flywheelDownVelDutycycle = new VelocityVoltage(0);
  private VelocityVoltage triggerDutyCycle = new VelocityVoltage(0);
  // private DigitalInput triggerSensorLeft = new DigitalInput(2);
  // private DigitalInput triggerSensorRight = new DigitalInput(3);

  public double desireVelocity = 0;
  private Timer ellapsedTime_Reset_trigger = new Timer();
  public boolean flag = false;
  
  // private static ShootingSubsystem instance;
  // public static ShootingSubsystem getInstance(){
  //   if(instance == null){
  //     instance = new ShootingSubsystem();
  //   }
  //   return instance;
  // }


  public Shooter(ShootTrigger strigger) {
     /* Motor Inverts and Neutral Mode */
    this.s_trigger = strigger;
    flywheelUpconfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    flywheelUpconfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    flywheelUpconfig.MotorOutput.DutyCycleNeutralDeadband = 0.04;

    /* Current Limiting */
    flywheelUpconfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    flywheelUpconfig.CurrentLimits.SupplyCurrentLimit = 20;
    flywheelUpconfig.CurrentLimits.SupplyCurrentThreshold = 30;
    flywheelUpconfig.CurrentLimits.SupplyTimeThreshold = 0.02;

     /* Open and Closed Loop Ramping */
    flywheelUpconfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 2;
    flywheelUpconfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 1;

    flywheelUpconfig.Slot0.kP = Constants.Shooter.flywheelKP;
    flywheelUpconfig.Slot0.kI = Constants.Shooter.flywheelKI;
    flywheelUpconfig.Slot0.kD = Constants.Shooter.flywheelKD;
    flywheelUpconfig.Slot0.kV = Constants.Shooter.flywheelKV;
    flywheelUpconfig.Slot0.kS = Constants.Shooter.flywheelKS;
    flywheelUpconfig.Slot0.kA = Constants.Shooter.flywheelKA;

    flywheelUpMotor.getConfigurator().apply(flywheelUpconfig);


    /* Motor Inverts and Neutral Mode */
    flywheelDownconfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    flywheelDownconfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    flywheelDownconfig.MotorOutput.DutyCycleNeutralDeadband = 0.04;

    /* Current Limiting */
    flywheelDownconfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    flywheelDownconfig.CurrentLimits.SupplyCurrentLimit = 20;
    flywheelDownconfig.CurrentLimits.SupplyCurrentThreshold = 30;
    flywheelDownconfig.CurrentLimits.SupplyTimeThreshold = 0.02;

     /* Open and Closed Loop Ramping */
    flywheelDownconfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 2;
    flywheelDownconfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 1;

    flywheelDownconfig.Slot0.kP = Constants.Shooter.flywheelKP;
    flywheelDownconfig.Slot0.kI = Constants.Shooter.flywheelKI;
    flywheelDownconfig.Slot0.kD = Constants.Shooter.flywheelKD;
    flywheelDownconfig.Slot0.kV = Constants.Shooter.flywheelKV;
    flywheelDownconfig.Slot0.kS = Constants.Shooter.flywheelKS;
    flywheelDownconfig.Slot0.kA = Constants.Shooter.flywheelKA;
    flywheelDownMotor.getConfigurator().apply(flywheelDownconfig);

    ellapsedTime_Reset_trigger.start();
  }

  public enum ShooterState{
    Stop,
    // Autonomous_shooting,
    // Auto_shooting,
    Default_shooting,
    Shootout, 
    ShootAmp,
    IDLE,
    Coasting,
    ShootingSpeaker, 
    ShootingMedium, 
    ShootingFar, 
    ShootingTower,
        // Get_prepare,
    // Source,
    // Ground_intake,
    // Speaker,
    // Speaker2,
    // Amp,
    // Trap,
    // SpitNorth

  }

  public enum FlywheelState{
    Stop,
    IDLE,
    Coasting,
    ShootAmp,
    ShootingSpeaker, 

    // Auto_Speed,
    // Get_prepare,
    // Source, 
    // Autonomous_shooting,
    // Speaker,
    // Speaker2,
    // Amp,
    // Trap
  }

  public ShooterState shooterState = ShooterState.Stop;
  public FlywheelState flywheelState = FlywheelState.Stop;
  
  public void update(){
    this.update(this.shooterState);
  }

  int coastingCount = 0;
  public void update(ShooterState state){
    this.shooterState = state;
    switch(state){
      case Stop:
        update(FlywheelState.Stop);
        this.s_trigger.update(TriggerState.Stop);
        this.shooterState = ShooterState.IDLE;
      break;
      case IDLE:

      break;
      case ShootingSpeaker:
        update(FlywheelState.ShootingSpeaker);
      break;
      case Coasting:
        coastingCount++;
        SmartDashboard.putNumber("coasting count", coastingCount);
        update(FlywheelState.Coasting);
      break;
      case Shootout:
        update(FlywheelState.ShootingSpeaker);
        if(Math.abs(flywheelUpMotor.getVelocity().getValue()/Constants.Shooter.shootingSpeaker)
          >Constants.Shooter.flywheelTolerance
        ){
          if(ellapsedTime_Reset_trigger.get()>2){
            flag=true;
            this.s_trigger.update(TriggerState.Shootout);
            this.s_trigger.setNoteReady(false);
            ellapsedTime_Reset_trigger.reset();
          }else{
            if(ellapsedTime_Reset_trigger.get()>1.5){
              update(ShooterState.Coasting);
              this.s_trigger.update(TriggerState.Stop);
            }
          }
        }else{
          if(ellapsedTime_Reset_trigger.get()>2){
          }else{
            update(ShooterState.Coasting);
            this.s_trigger.update(TriggerState.Stop);
          }
        }

      break;
      case ShootAmp:
        this.flywheelState = FlywheelState.ShootAmp;
        if(Math.abs(flywheelUpMotor.getVelocity().getValue()/Constants.Shooter.AmpUpSpeed)
          >Constants.Shooter.flywheelTolerance
        ){
          if(ellapsedTime_Reset_trigger.get()>2){
            flag=true;
            this.s_trigger.update(TriggerState.Shootout);
            this.s_trigger.setNoteReady(false);
            ellapsedTime_Reset_trigger.reset();
          }else{
            if(ellapsedTime_Reset_trigger.get()>1.5){
              update(ShooterState.Coasting);
              this.s_trigger.update(TriggerState.Stop);
            }
          }
        }else{
          if(ellapsedTime_Reset_trigger.get()>2){
          }else{
            update(ShooterState.Coasting);
            this.s_trigger.update(TriggerState.Stop);
          }
        }

      break;

      default:
      break;
    }

  }

  public void update(FlywheelState state){
    this.flywheelState = state;
    switch(state){
      case Stop:
        stopflywheel();
        this.flywheelState = FlywheelState.IDLE;
      break;
      case IDLE:
        this.flywheelState = FlywheelState.Coasting;
      break;
      case Coasting:
        flywheelControl(Constants.Shooter.shootingCoasting);
      break;
      case ShootingSpeaker:
        flywheelControl(Constants.Shooter.shootingSpeaker);
      break;
      case ShootAmp:
        flywheelAmpControl();
      break;
      default:
      break;
    }

  }


  public void flywheelControl(double velocity){
    flywheelUpVelDutycycle.Velocity = velocity;
    flywheelUpMotor.setControl(flywheelUpVelDutycycle);
    flywheelDownVelDutycycle.Velocity = velocity;
    flywheelDownMotor.setControl(flywheelUpVelDutycycle);
  }

  public void flywheelAmpControl(){
    flywheelUpVelDutycycle.Velocity = Constants.Shooter.AmpUpSpeed;
    flywheelUpMotor.setControl(flywheelUpVelDutycycle);
    flywheelDownVelDutycycle.Velocity = Constants.Shooter.AmpDownSpeed;
    flywheelDownMotor.setControl(flywheelDownVelDutycycle);
  }

  public double getFlywheelVelocity(){
    return flywheelUpMotor.getVelocity().getValue();
  }

  public double getShooterVelocity(){
    return flywheelUpMotor.getVelocity().getValue();
  }

  public void stopflywheel(){
    flywheelUpMotor.stopMotor();
    flywheelDownMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // SmartDashboard.putBoolean("shooterSensor0", triggerSensorLeft.get());
    SmartDashboard.putNumber("ellapsedTime_Reset_trigger", ellapsedTime_Reset_trigger.get());
    SmartDashboard.putNumber("ShooterVelocity", getShooterVelocity());
    SmartDashboard.putString("shooterState", this.shooterState.toString());
    SmartDashboard.putString("flywheelState", this.flywheelState.toString());
    SmartDashboard.putNumber("flywheelLeftMotor getVelocity", flywheelUpMotor.getVelocity().getValue());
    SmartDashboard.putBoolean("flagboolean", flag);

  }
}
