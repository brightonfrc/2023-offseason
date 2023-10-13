// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */


public class Robot extends TimedRobot {
  
  private final PWMSparkMax m_leftDrive = new PWMSparkMax(0);
  private final PWMSparkMax m_rightDrive = new PWMSparkMax(1);
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftDrive, m_rightDrive);
  private final XboxController m_controller = new XboxController(0);
  private final Timer m_timer = new Timer();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightDrive.setInverted(true);
  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    // creating the variables required for the swerve drive odometry class
    // locations for motors relative to center of the robot
    Translation2d m_frontLeftLocation = new Translation2d();
    Translation2d m_frontRightLocation = new Translation2d();
    Translation2d m_backLeftLocation = new Translation2d();
    Translation2d m_backRightLocation = new Translation2d();
    SwerveModulePosition m_frontLeft = new SwerveModulePosition();
    SwerveModulePosition m_frontRight = new SwerveModulePosition();
    SwerveModulePosition m_backLeft = new SwerveModulePosition();
    SwerveModulePosition m_backRight = new SwerveModulePosition();
    SwerveModulePosition[] MotorPositions= {m_frontLeft,m_frontRight,m_backLeft,m_backRight};
    //initialising the gyro
    WPI_PigeonIMU gyro = new WPI_PigeonIMU(0); 
    //initialising the swerve drive kinematics
    SwerveDriveKinematics robot_kinematics= new SwerveDriveKinematics(m_frontLeftLocation,m_frontRightLocation,m_backLeftLocation,m_backRightLocation);
    // initial coordinate and bearing of the robot
    Pose2d currentPose = new Pose2d();
    // creating the SwerveDriveOdometry
    SwerveDriveOdometry odometry = new SwerveDriveOdometry(robot_kinematics, gyro.getRotation2d(),MotorPositions,currentPose);
    m_timer.restart();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    //find a way to get this function to recognise variables declared in autonomousInit()
  }

  /** This function is called once each time the robot enters teleoperated mode. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
    m_robotDrive.arcadeDrive(-m_controller.getLeftY(), -m_controller.getRightX());
  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
