// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.WPILibVersion;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubSystem extends SubsystemBase {
  /** Creates a new DriveSubSystem. */
  private WPI_TalonFX leftDrivePrimary = new WPI_TalonFX(0);
  private WPI_TalonFX leftDriveFollower1 = new WPI_TalonFX(1);
  private WPI_TalonFX leftDriveFollower2 = new WPI_TalonFX(2);

  private WPI_TalonFX rightDrivePrimary = new WPI_TalonFX(3);
  private WPI_TalonFX rightDriveFollower1 = new WPI_TalonFX(4);
  private WPI_TalonFX rightDriveFollower2 = new WPI_TalonFX(5);

  public DriveSubSystem() {
    leftDrivePrimary.configFactoryDefault();
    leftDriveFollower1.configFactoryDefault();
    leftDriveFollower2.configFactoryDefault();
    rightDrivePrimary.configFactoryDefault();
    rightDriveFollower1.configFactoryDefault();
    rightDriveFollower2.configFactoryDefault();

    leftDriveFollower1.follow(leftDrivePrimary);
    leftDriveFollower2.follow(leftDrivePrimary);

    rightDriveFollower1.follow(rightDrivePrimary);
    rightDriveFollower2.follow(rightDrivePrimary);

    leftDrivePrimary.setInverted(InvertType.InvertMotorOutput);
    leftDriveFollower1.setInverted(InvertType.FollowMaster);
    leftDriveFollower2.setInverted(InvertType.FollowMaster);
    
    rightDrivePrimary.setInverted(InvertType.None);
    rightDriveFollower1.setInverted(InvertType.FollowMaster);
    rightDriveFollower2.setInverted(InvertType.FollowMaster);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("left motor speed", leftDrivePrimary.getSelectedSensorVelocity());
  }

  public void set(double left_speed, double  right_speed) {
    leftDrivePrimary.set(ControlMode.PercentOutput, left_speed);
    rightDrivePrimary.set(ControlMode.PercentOutput, right_speed);
  }
  
  public void stop() {
    leftDrivePrimary.set(ControlMode.PercentOutput, 0);
    rightDrivePrimary.set(ControlMode.PercentOutput, 0);
  }
}
