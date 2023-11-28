// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.I2C;
import com.kauailabs.navx.frc.AHRS;



public class Drivetrain extends SubsystemBase {
  private Translation2d m_frontLeftLocation = new Translation2d();
  private Translation2d m_frontRightLocation = new Translation2d();
  private Translation2d m_backLeftLocation = new Translation2d();
  private Translation2d m_backRightLocation = new Translation2d();
  private SwerveModulePosition m_frontLeft = new SwerveModulePosition();
  private SwerveModulePosition m_frontRight = new SwerveModulePosition();
  private SwerveModulePosition m_backLeft = new SwerveModulePosition();
  private SwerveModulePosition m_backRight = new SwerveModulePosition();
  private SwerveModulePosition[] MotorPositions= {m_frontLeft,m_frontRight,m_backLeft,m_backRight};
  //initialising the gyro
  //remember to update where it is connected to
  private AHRS gyro = new AHRS(I2C.Port.kMXP);
  //initialising the swerve drive kinematics
  private SwerveDriveKinematics robot_kinematics= new SwerveDriveKinematics(m_frontLeftLocation,m_frontRightLocation,m_backLeftLocation,m_backRightLocation);
  // initial coordinate and bearing of the robot
  private Pose2d currentPose = new Pose2d();
  //presumably, the start bearing can be considered 0
  private Rotation2d convertedAngle= new Rotation2d(0);
  // creating the SwerveDriveOdometry
  private SwerveDriveOdometry odometry = new SwerveDriveOdometry(robot_kinematics, convertedAngle,MotorPositions,currentPose);

  // creating the variables required for the autonomous code
  private double bearing;
  private double TargetBearing;
  private double position;
  private double[] destination ={3.0,3.0};
  private double previous;
  private double elapsed;
  private SwerveModulePosition NewPosition= new SwerveModulePosition();
  private Rotation2d NewModuleAngle=new Rotation2d();
  //setting distance tolerance to 1cm
  private double dTolerance = 0.01;
  //setting bearing tolerance to 0.5 degrees
  private double bTolerance=0.5;
  //remember to tune the PID controller
  private double bearingP;
  private double bearingD;
  private double bearingI;
  private PIDController bearingError = new PIDController(bearingP,bearingI,bearingD);
  private double bearingOutput;
  private double distanceP;
  private double distanceD;
  private double distanceI;
  private PIDController distanceError = new PIDController(distanceP,distanceI,distanceD);
  private double positionOutput;
  /** Creates a new Drivetrain. */
  public Drivetrain() {
    
  }
  public void setMotor(int PIDoutput){

  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
