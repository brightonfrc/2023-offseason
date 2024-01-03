// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveForwardsCommand extends CommandBase {
  private DriveSubsystem drivetrain;
  private long startTime;

  /** Creates a new DriveForwardsCommand. */
  public DriveForwardsCommand(DriveSubsystem drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    
    this.addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.drivetrain.driveWithSpeeds(1, 1);
    this.startTime = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.drivetrain.driveWithSpeeds(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    long millisecondsSoFar = System.currentTimeMillis() - this.startTime;
    return millisecondsSoFar > 1000L; // L means long so types are the same; taken more than 1 second
  }
}
