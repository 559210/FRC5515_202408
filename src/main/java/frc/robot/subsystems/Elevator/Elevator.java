// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public Elevator() {
    configElevator();
  }

  public static final TalonFX Elevator = new TalonFX(Constants.Elevator.elevatorMotorID,"rio");

  public MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);

  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  private TalonFXConfiguration ElevatorConfiguration() {
    TalonFXConfiguration elevatorConfiguration = new TalonFXConfiguration();
    elevatorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    elevatorConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 296;
    elevatorConfiguration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 4;
    elevatorConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    elevatorConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    elevatorConfiguration.Slot0.kP = Constants.Elevator.KP;
    elevatorConfiguration.Slot0.kI = Constants.Elevator.KI;
    elevatorConfiguration.Slot0.kD = Constants.Elevator.KD;
    elevatorConfiguration.Slot0.kS = Constants.Elevator.KS;
    elevatorConfiguration.Slot0.kV = Constants.Elevator.KV;

    elevatorConfiguration.MotionMagic.MotionMagicCruiseVelocity = Constants.Elevator.Velocity;
    elevatorConfiguration.MotionMagic.MotionMagicAcceleration = Constants.Elevator.Acceleration;
    elevatorConfiguration.MotionMagic.MotionMagicJerk = Constants.Elevator.Jerk;
    return elevatorConfiguration;
  }

  private void configElevator() {
    // Elevator.getConfigurator().apply(new TalonFXConfiguration());
    Elevator.getConfigurator().apply(ElevatorConfiguration());
  }

  public void elevatorUp() {
    motionMagicVoltage.Position = Constants.Elevator.Top;
    Elevator.setControl(motionMagicVoltage);
  }

  public void elevatorDown() {
    motionMagicVoltage.Position = Constants.Elevator.Bottom;
    Elevator.setControl(motionMagicVoltage);
  }

  // public void elevator() {
  //   if (elevatorTop)
  //     elevatorDown();
  //   else
  //     elevatorUp();
  // }
}
