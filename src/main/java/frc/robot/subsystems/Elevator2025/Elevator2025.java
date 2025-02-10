package frc.robot.subsystems.Elevator2025;

import java.text.BreakIterator;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants2025;

public class Elevator2025 extends SubsystemBase {
    public enum EV_STATE {
        NONE,
        ZERO,
        BASE,

    }

    EV_STATE curState = EV_STATE.NONE;
    /** Creates a new ExampleSubsystem. */
    public Elevator2025() {
    }

    public static final TalonFX m_primaryMotor = new TalonFX(Constants2025.Elevator.primaryMotorID, Constants2025.Elevator.canBusName);
    public static final TalonFX m_followerMotor = new TalonFX(Constants2025.Elevator.followerMotorID, Constants2025.Elevator.canBusName);
    public static final CANcoder m_canCoder = new CANcoder(Constants2025.Elevator.canCoderID, Constants2025.Elevator.canBusName);
    public MotionMagicVoltage motionMagicVoltage1 = new MotionMagicVoltage(0);
    public MotionMagicVoltage motionMagicVoltage2 = new MotionMagicVoltage(1);

    int runingCount = 0;
    @Override
    public void periodic() {
        runingCount++;
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Elevator run", runingCount);
        // SmartDashboard.putBoolean("Elevator Top", elevatorTop());
        updateState();
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    private TalonFXConfiguration getMotorConfiguration(boolean isPrimary) {
        TalonFXConfiguration elevatorConfiguration = new TalonFXConfiguration();

        // 一正一反？
        // 问题： CanCoder到底是哪个Motor的读数？
        elevatorConfiguration.MotorOutput.Inverted = isPrimary ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        elevatorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        // elevatorConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0;
        // elevatorConfiguration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -200;
        // elevatorConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        // elevatorConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        elevatorConfiguration.Slot0.kP = Constants2025.Elevator.KP;
        elevatorConfiguration.Slot0.kI = Constants2025.Elevator.KI;
        elevatorConfiguration.Slot0.kD = Constants2025.Elevator.KD;
        elevatorConfiguration.Slot0.kS = Constants2025.Elevator.KS;
        elevatorConfiguration.Slot0.kV = Constants2025.Elevator.KV;
        elevatorConfiguration.Slot0.kA = Constants2025.Elevator.KA;

        elevatorConfiguration.MotionMagic.MotionMagicCruiseVelocity = Constants2025.Elevator.Velocity;
        elevatorConfiguration.MotionMagic.MotionMagicAcceleration = Constants2025.Elevator.Acceleration;
        elevatorConfiguration.MotionMagic.MotionMagicJerk = Constants2025.Elevator.Jerk;
        
        elevatorConfiguration.Feedback.SensorToMechanismRatio = 8.8; // 8.8 到底是 SensorToMechanismRatio 还是 RotorToSensorRatio？
        if (isPrimary) {
            elevatorConfiguration.Feedback.FeedbackRemoteSensorID = Constants2025.Elevator.canCoderID;
            elevatorConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        }

        // 似乎只有 FeedbackSensorSourceValue.FusedCANcoder 模式下才需要下面两个
        // elevatorConfiguration.Feedback.SensorToMechanismRatio = 1.0;
        // elevatorConfiguration.Feedback.RotorToSensorRatio = 12.8;
        return elevatorConfiguration;
    }

    private CANcoderConfiguration getCCConfig() {
        CANcoderConfiguration cc_cfg = new CANcoderConfiguration();
        // cc_cfg.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        cc_cfg.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        cc_cfg.MagnetSensor.MagnetOffset = 0;

        return cc_cfg;
    }

    public void init() {
        SmartDashboard.putNumber("ccc init", m_canCoder.getPosition().getValueAsDouble());
        // Elevator.getConfigurator().apply(new TalonFXConfiguration());
        m_primaryMotor.getConfigurator().apply(getMotorConfiguration(true));
        m_followerMotor.getConfigurator().apply(getMotorConfiguration(false));

        m_followerMotor.setControl(new Follower(m_primaryMotor.getDeviceID(), true));
        // m_primaryMotor.setSafetyEnabled(true);      // 例子代码，不知道有什么用，不明白safetyEnabled是什么
        m_canCoder.getConfigurator().apply(getCCConfig());

    
        motionMagicVoltage1 = new MotionMagicVoltage(m_canCoder.getPosition().getValueAsDouble());
        motionMagicVoltage2 = new MotionMagicVoltage(m_canCoder.getPosition().getValueAsDouble());
        setState(EV_STATE.ZERO);
    }

    public void setState(EV_STATE stat) {
        curState = stat;
    }

    protected void updateState() {
        SmartDashboard.putNumber("ccc1", m_canCoder.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("ccc2", m_canCoder.getAbsolutePosition().getValueAsDouble());
        double pos = Constants2025.Elevator.basePos;
        switch (curState) {
            case ZERO:
                pos = Constants2025.Elevator.zeroPos;
                break;
            case BASE:
                pos = Constants2025.Elevator.basePos;
                break;
            case NONE:
                return;
            default:
                break;
        }
        // m_canCoder.getAbsolutePosition()
        // m_canCoder.setPosition(pos);
        // m_canCoder.setControl(motionMagicVoltage.withPosition(pos));
        m_primaryMotor.setControl(motionMagicVoltage1.withPosition(pos).withSlot(0));
        // m_followerMotor.setControl(motionMagicVoltage2.withPosition(pos));
    }

    // int upC = 0;
    // int dC = 0;

    // public void elevatorUp() {
    //     upC++;
    //     SmartDashboard.putNumber("EUp", upC);
    //     // motionMagicVoltage.Position = Constants2025.Elevator.Top;
    //     m_arm.setControl(motionMagicVoltage.withPosition(Constants2025.TurningArm.Top));
    // }

    // public void elevatorDown() {
    //     dC++;
    //     SmartDashboard.putNumber("EDown", dC);
    //     // motionMagicVoltage.Position = Constants2025.Elevator.Bottom;
    //     m_arm.setControl(motionMagicVoltage.withPosition(Constants2025.TurningArm.Bottom));
    // }

    // public static boolean elevatorTop() {
    //     if (m_arm.getPosition().getValueAsDouble() == Constants2025.TurningArm.Top)
    //         return true;
    //     else
    //         return false;
    // }
}
