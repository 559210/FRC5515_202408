package frc.robot.subsystems.Elevator2025;

import java.text.BreakIterator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

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

    public static final TalonFX m_motor1 = new TalonFX(Constants2025.Elevator.motorID1, Constants2025.Elevator.canBusName);
    public static final TalonFX m_motor2 = new TalonFX(Constants2025.Elevator.motorID2, Constants2025.Elevator.canBusName);
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

    private TalonFXConfiguration ElevatorConfiguration() {
        TalonFXConfiguration elevatorConfiguration = new TalonFXConfiguration();
        // elevatorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
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
        
        // elevatorConfiguration.Feedback.SensorToMechanismRatio = 8.8;
        elevatorConfiguration.Feedback.FeedbackRemoteSensorID = Constants2025.Elevator.canCoderID;
        elevatorConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        return elevatorConfiguration;
    }

    public void init() {
        SmartDashboard.putNumber("ccc init", m_canCoder.getPosition().getValueAsDouble());
        // Elevator.getConfigurator().apply(new TalonFXConfiguration());
        m_motor1.getConfigurator().apply(ElevatorConfiguration());
        m_motor2.getConfigurator().apply(ElevatorConfiguration());
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
        m_motor1.setControl(motionMagicVoltage1.withPosition(pos));
        m_motor2.setControl(motionMagicVoltage2.withPosition(pos));
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
