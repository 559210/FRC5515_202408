// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Elevator.Elevator;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ElevatorCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Elevator elevatorSubsystem;
  private final boolean up;
  // private final ShooterSubsystem shooterSubsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ElevatorCommand(Elevator elevatorSubsystem, boolean up) {
    this.up = up;
    this.elevatorSubsystem = elevatorSubsystem;
    // this.shooterSubsystem = shooterSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    // shooterSubsystem.VerticalRotation(ShooterConstants.LowestPosition);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (up)
      elevatorSubsystem.elevatorUp();
    else
      elevatorSubsystem.elevatorDown();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // ElevatorSubsystem.Elevator.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
  //   if (ElevatorSubsystem.elevatorTop()) 
  //     return true;
  //   else
      return false;
  }
}
