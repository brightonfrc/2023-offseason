// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubSystem;

public class DefaultDriveCommand extends CommandBase {

  private DriveSubSystem DRIVE_SUBSYSTEM;
  private Joystick controller;
  
  double left;
  double right;

  SlewRateLimiter left_limiter = new SlewRateLimiter(1);
  SlewRateLimiter right_limiter = new SlewRateLimiter(1);

  public DefaultDriveCommand(DriveSubSystem drive, Joystick con) {
    this.DRIVE_SUBSYSTEM = drive;
    this.controller = con;

    addRequirements(DRIVE_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    left = 0;
    right = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    left = controller.getRawAxis(1); // left y axis
    right = controller.getRawAxis(5); // right y axis

    DRIVE_SUBSYSTEM.set(left_limiter.calculate(left), right_limiter.calculate(right));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DRIVE_SUBSYSTEM.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
