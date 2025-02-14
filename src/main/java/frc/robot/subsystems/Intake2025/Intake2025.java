package frc.robot.subsystems.Intake2025;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.CanID;
// import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants;
import frc.robot.Constants2025;
import frc.robot.StateController;
import frc.robot.subsystems.Trigger.ShootTrigger;
import frc.robot.subsystems.Trigger.ShootTrigger.TriggerState;
import frc.robot.utils.MiscUtils;

public class Intake2025 extends SubsystemBase {
    public static final TalonFX m_motor = new TalonFX(Constants2025.Intake.motorID,
            Constants2025.Intake.canBusName);
    private VelocityVoltage intakeVelDutycycle = new VelocityVoltage(0);

    private DigitalInput intakeCoralSensor = new DigitalInput(0);
    private boolean isIntakeCoralSensorOn = false;
    private int intakeCoralSensorOnTickCount = -1;
    private int intakeCoralSensorOffTickCount = -1;
    private final int intakeCoralSensorOnTickCountThreshold = 50;       // a tick is about 1/50 second(50Hz)
    private final int intakeCoralSensorOffTickCountThreshold = 50;      // a tick is about 1/50 second(50Hz)

    public enum STATE {
        READY,
        CORAL_IN,
        CARRYING_CORAL,
        CORAL_OUT,
        BALL_IN,
        BALL_OUT,
        CARRYING_BALL,
    }

    private STATE curState = STATE.READY;

    public Intake2025() {
    }

    private TalonFXConfiguration getMotorConfiguration() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        /* Current Limiting */
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 20;
        config.CurrentLimits.SupplyCurrentLowerLimit = 30;
        config.CurrentLimits.SupplyCurrentLowerTime = 0.02;

        
        /* Open and Closed Loop Ramping */
        config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 2;
        config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 1;


        config.Slot0.kP = Constants2025.Intake.KP;
        config.Slot0.kI = Constants2025.Intake.KI;
        config.Slot0.kD = Constants2025.Intake.KD;
        config.Slot0.kS = Constants2025.Intake.KS;
        config.Slot0.kV = Constants2025.Intake.KV;
        config.Slot0.kA = Constants2025.Intake.KA;

        return config;
    }

    public void init() {
        m_motor.getConfigurator().apply(getMotorConfiguration());
        setState(STATE.READY);
    }

    public void toggleCoralIntake() {
        if (this.curState == STATE.READY) {
            setState(STATE.CORAL_IN);
        }

        if (this.curState == STATE.CORAL_IN) {
            setState(STATE.READY);
        }

        if (this.curState == STATE.CARRYING_CORAL) {
            setState(STATE.CORAL_OUT);
        }
    }

    public void toggleBallIntake() {
        if (this.curState == STATE.READY) {
            setState(STATE.BALL_IN);
        }

        if (this.curState == STATE.BALL_IN) {
            setState(STATE.READY);
        }
        if (this.curState == STATE.CARRYING_BALL) {
            setState(STATE.BALL_OUT);
        }
    }

    public void setState(STATE st) {
        this.curState = st;
    }

    private boolean getIsCarryingCarol() {
        return isIntakeCoralSensorOn;
    }

    private void updateIsCarryingCarol() {
        boolean isCarrying = !intakeCoralSensor.get();
        if (isCarrying) {
            intakeCoralSensorOffTickCount = -1;
            if (intakeCoralSensorOnTickCount == -1) {
                intakeCoralSensorOnTickCount = 0;
            }
            else {
                intakeCoralSensorOnTickCount++;
            }
            if (intakeCoralSensorOnTickCount > intakeCoralSensorOnTickCountThreshold) {
                isIntakeCoralSensorOn = true;
            }
        }
        else {
            intakeCoralSensorOnTickCount = -1;
            if (intakeCoralSensorOffTickCount == -1) {
                intakeCoralSensorOffTickCount = 0;
            }
            else {
                intakeCoralSensorOffTickCount++;
            }
            if (intakeCoralSensorOffTickCount > intakeCoralSensorOffTickCountThreshold) {
                isIntakeCoralSensorOn = false;
            }
        }
    };

    private void updateState() {
        double speed = 0;
        switch (curState) {
            case READY:
            case CARRYING_BALL:
            case CARRYING_CORAL:
                speed = 0;
                break;
            case CORAL_IN:
                if (getIsCarryingCarol()) {
                    curState = STATE.CARRYING_CORAL;
                }
                else {
                    speed = Constants2025.Intake.coralInSpeed;
                }
                break;
            case CORAL_OUT:
                if (!getIsCarryingCarol()) {
                    curState = STATE.READY;
                }
                else {
                    speed = Constants2025.Intake.coralOutSpeed;
                }
                break;
            case BALL_IN:
                speed = Constants2025.Intake.BallInSpeed;
                break;
            case BALL_OUT:
                speed = Constants2025.Intake.BAllOutSpeed;
                break;
            default:
                break;
        }

        if (MiscUtils.compareDouble(speed, 0)) {
            m_motor.stopMotor();
        }
        else {
            intakeVelDutycycle.Velocity = speed;
            m_motor.setControl(intakeVelDutycycle);
        }
    }

    @Override
    public void periodic() {
        updateIsCarryingCarol();
        updateState();
    }
}
