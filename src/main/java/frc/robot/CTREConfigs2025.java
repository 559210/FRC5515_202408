package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public final class CTREConfigs2025 {
    public TalonFXConfiguration swerveAngleFXConfig = new TalonFXConfiguration();
    public TalonFXConfiguration swerveDriveFXConfig = new TalonFXConfiguration();
    public CANcoderConfiguration swerveCANcoderConfig = new CANcoderConfiguration();

    public CTREConfigs2025(int moduletype){
        /** Swerve CANCoder Configuration */
        if (moduletype == 0) {
            swerveCANcoderConfig.MagnetSensor.SensorDirection = Constants2025.Swerve.cancoderInvert;
        }
        else {
            swerveCANcoderConfig.MagnetSensor.SensorDirection = Constants2025.Swerve.cancoderInvert;
        }

        /** Swerve Angle Motor Configurations */
        /* Motor Inverts and Neutral Mode */
        swerveAngleFXConfig.MotorOutput.NeutralMode = Constants2025.Swerve.angleNeutralMode;

        /* Gear Ratio and Wrapping Config */
        swerveAngleFXConfig.Feedback.SensorToMechanismRatio = Constants2025.Swerve.angleGearRatio;
        swerveAngleFXConfig.ClosedLoopGeneral.ContinuousWrap = true;
        
        /* Current Limiting */
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants2025.Swerve.angleEnableCurrentLimit;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimit = Constants2025.Swerve.angleCurrentLimit;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLowerLimit = Constants2025.Swerve.angleCurrentThreshold;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLowerTime = Constants2025.Swerve.angleCurrentThresholdTime;

        /* PID Config */
        swerveAngleFXConfig.Slot0.kP = Constants2025.Swerve.angleKP;
        swerveAngleFXConfig.Slot0.kI = Constants2025.Swerve.angleKI;
        swerveAngleFXConfig.Slot0.kD = Constants2025.Swerve.angleKD;

        /** Swerve Drive Motor Configuration */
        /* Motor Inverts and Neutral Mode */
        swerveDriveFXConfig.MotorOutput.Inverted = Constants2025.Swerve.driveMotorInvert;
        if (moduletype==0){
            swerveDriveFXConfig.MotorOutput.Inverted = Constants2025.Swerve.driveMotorInvert;
            swerveAngleFXConfig.MotorOutput.Inverted = Constants2025.Swerve.angleMotorInvert;
        }else{
            swerveDriveFXConfig.MotorOutput.Inverted = Constants2025.Swerve.driveMotorInvert2;
            swerveAngleFXConfig.MotorOutput.Inverted = Constants2025.Swerve.angleMotorInvert;
        }
        swerveDriveFXConfig.MotorOutput.NeutralMode = Constants2025.Swerve.driveNeutralMode;

        /* Gear Ratio Config */
        swerveDriveFXConfig.Feedback.SensorToMechanismRatio = Constants2025.Swerve.driveGearRatio;

        /* Current Limiting */
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants2025.Swerve.driveEnableCurrentLimit;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimit = Constants2025.Swerve.driveCurrentLimit;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLowerLimit = Constants2025.Swerve.driveCurrentThreshold;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLowerTime = Constants2025.Swerve.driveCurrentThresholdTime;

        /* PID Config */
        swerveDriveFXConfig.Slot0.kP = Constants2025.Swerve.driveKP;
        swerveDriveFXConfig.Slot0.kI = Constants2025.Swerve.driveKI;
        swerveDriveFXConfig.Slot0.kD = Constants2025.Swerve.driveKD;

        /* Open and Closed Loop Ramping */
        swerveDriveFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Constants2025.Swerve.openLoopRamp;
        swerveDriveFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = Constants2025.Swerve.openLoopRamp;

        swerveDriveFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = Constants2025.Swerve.closedLoopRamp;
        swerveDriveFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants2025.Swerve.closedLoopRamp;
    }
}