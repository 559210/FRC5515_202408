// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.TurningArm2025;

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

public class TurningArm2025 extends SubsystemBase {
    public enum TA_STATE {
        NONE,
        ZERO,
        BASE,

    }

    TA_STATE curState = TA_STATE.NONE;
    /** Creates a new ExampleSubsystem. */
    public TurningArm2025() {
    }

    public static final TalonFX m_arm = new TalonFX(Constants2025.TurningArm.motorID, Constants2025.TurningArm.canBusName);
    public static final CANcoder m_canCoder = new CANcoder(Constants2025.TurningArm.canCoderID, Constants2025.TurningArm.canBusName);
    public MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);

    int runingCount = 0;
    @Override
    public void periodic() {
        runingCount++;
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("TurningArm run", runingCount);
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

        elevatorConfiguration.Slot0.kP = Constants2025.TurningArm.KP;
        elevatorConfiguration.Slot0.kI = Constants2025.TurningArm.KI;
        elevatorConfiguration.Slot0.kD = Constants2025.TurningArm.KD;
        elevatorConfiguration.Slot0.kS = Constants2025.TurningArm.KS;
        elevatorConfiguration.Slot0.kV = Constants2025.TurningArm.KV;
        elevatorConfiguration.Slot0.kA = Constants2025.TurningArm.KA;

        elevatorConfiguration.MotionMagic.MotionMagicCruiseVelocity = Constants2025.TurningArm.Velocity;
        elevatorConfiguration.MotionMagic.MotionMagicAcceleration = Constants2025.TurningArm.Acceleration;
        elevatorConfiguration.MotionMagic.MotionMagicJerk = Constants2025.TurningArm.Jerk;
        
        elevatorConfiguration.Feedback.SensorToMechanismRatio = 28 / 3;
        elevatorConfiguration.Feedback.FeedbackRemoteSensorID = Constants2025.TurningArm.canCoderID;
        elevatorConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        return elevatorConfiguration;
    }

    public void init() {
        // Elevator.getConfigurator().apply(new TalonFXConfiguration());
        m_arm.getConfigurator().apply(ElevatorConfiguration());
        setState(TA_STATE.ZERO);
    }

    public void setState(TA_STATE stat) {
        curState = stat;
    }

    protected void updateState() {
        double pos = Constants2025.TurningArm.basePos;
        switch (curState) {
            case ZERO:
                pos = Constants2025.TurningArm.zeroPos;
                break;
            case BASE:
                pos = Constants2025.TurningArm.basePos;
                break;
            case NONE:
                return;
            default:
                break;
        }
        // m_canCoder.getAbsolutePosition()
        // m_canCoder.setPosition(pos);
        // m_canCoder.setControl(motionMagicVoltage.withPosition(pos));
        m_arm.setControl(motionMagicVoltage.withPosition(pos));
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
