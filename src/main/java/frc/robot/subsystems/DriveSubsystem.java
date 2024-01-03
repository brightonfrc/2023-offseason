// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  WPI_TalonFX leftMotor;
  WPI_TalonFX rightMotor;
  
  /** Creates a new DriveSubsystem. */
  public DriveSubsystem(WPI_TalonFX leftMotor, WPI_TalonFX rightMotor) {
    this.leftMotor = leftMotor;
    rightMotor.setInverted(true); // if needed as wrong way round
    this.rightMotor = rightMotor;
  }

  public void driveWithSpeeds(double left, double right) {
    this.leftMotor.set(left);
    this.rightMotor.set(right);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
