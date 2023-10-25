// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
import edu.wpi.first.wpilibj.I2C;
import com.kauailabs.navx.frc.AHRS;



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

  // creating the variables required for the swerve drive odometry class
  // remember to modify locations for motors relative to center of the robot
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
    distanceError.setTolerance(dTolerance);
    //remember to update setpoint whenever a new destination has been reached
    distanceError.setSetpoint(Math.pow(Math.pow(destination[0],2)+Math.pow(destination[1],2),0.5));
    bearingError.setTolerance(bTolerance);
    //for somewhat obvious reasons, any bearings above or below this range will be converted to this range
    bearingError.enableContinuousInput(0, 360);
    m_timer.restart();
    //remember to call this line of code whenever a new destination has been reached
    TargetBearing= Math.tan((destination[0]-currentPose.getX())/(destination[1]-currentPose.getY()));
    bearingError.setSetpoint(TargetBearing);
    previous=m_timer.get();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    //updating the position of each swervedrive module
    for(int index=0;index<4;index++){
      //finding the time that has passed since the last call of autonomousPeriodic
      elapsed=m_timer.get()-previous;
      //get the velocity of each module and update their new positions accordingly. 
      //0 is just the stopgap values
      NewModuleAngle.fromDegrees(0.0);
      NewPosition= new SwerveModulePosition(0.0,NewModuleAngle);
      MotorPositions[index]=NewPosition;
      previous=m_timer.get();
    }
    //updating the current position of the robot
    convertedAngle=Rotation2d.fromDegrees(bearing);
    currentPose= odometry.update(convertedAngle,MotorPositions);
    position = Math.pow(Math.pow(currentPose.getX(),2)+Math.pow(currentPose.getY(),2),0.5);
    //updating the position PID
    positionOutput=distanceError.calculate(position);
    if(distanceError.atSetpoint()==false){
      //write code to modify the motor velocities
      
    } else{
      //updating the bearing PID
      bearingOutput=bearingError.calculate(gyro.getAngle()%360);
      if(bearingError.atSetpoint()){
        //put the function for the robot to score the point by putting stuff down(depends on the rules)
      }else{
      //write code to get the robot to turn towards the correct bearing so that it can score the points 
      
    }
    }
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
