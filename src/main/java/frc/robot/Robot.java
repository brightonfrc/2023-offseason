// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.AnalogInput;

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

  public DigitalOutput ultrasonicTriggerPinOne = new DigitalOutput(0);
  public DigitalOutput ultrasonicTriggerPinTwo = new DigitalOutput(1);
 
  public AnalogInput ultrasonicSensorOne = new AnalogInput(0);
  public AnalogInput ultrasonicSensorTwo = new AnalogInput(1);
 
  public double ultrasonicSensorOneRange = 0;
  public double ultrasonicSensorTwoRange = 0;
 
  public double voltageScaleFactor = 1;
 
  public void turnOnSensorOne() {
    ultrasonicTriggerPinOne.set(true);
    ultrasonicTriggerPinTwo.set(false);
  }
 
 /* public void turnOnSensorTwo() {
    ultrasonicTriggerPinOne.set(false);
    ultrasonicTriggerPinTwo.set(true);
  }
 
  public void turnOffBothSensors() {
    ultrasonicTriggerPinOne.set(false);
    ultrasonicTriggerPinTwo.set(false);
  } */

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

    //Initialize range readings on SmartDashboard as max distance in Centimeters.
    SmartDashboard.putNumber("Sensor 1 Range", 500);
    SmartDashboard.putNumber("Sensor 2 Range", 500);
  }

  @Override
  public void robotPeriodic() {
    //Publish range readings to SmartDashboard
    SmartDashboard.putNumber("Sensor 1 Range", ultrasonicSensorOneRange);
    SmartDashboard.putNumber("Sensor 2 Range", ultrasonicSensorTwoRange);
 
    voltageScaleFactor = 5/RobotController.getVoltage5V(); //Calculate what percentage of 5 Volts we are actually at
  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    m_timer.restart();
    //If we are in Autonomous mode, turn on the first sensor (and turn off the second sensor)
    turnOnSensorOne();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    //Get a reading from the first sensor, scale it by the voltageScaleFactor, and then scale to Centimeters
    ultrasonicSensorOneRange = ultrasonicSensorOne.getValue()*voltageScaleFactor*0.125;
    
    // Drive for 2 seconds
    if (m_timer.get() < 2.0) {
      // Drive forwards half speed, make sure to turn input squaring off
      m_robotDrive.arcadeDrive(0.5, 0.0, false);
    } else {
      m_robotDrive.stopMotor(); // stop robot
    }
  }

  /** This function is called once each time the robot enters teleoperated mode. */
  @Override
  public void teleopInit() {
    //If we are in Teleoperated mode, turn on the second sensor (and turn off the first sensor)
    turnOnSensorTwo();
  }

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
    m_robotDrive.arcadeDrive(-m_controller.getLeftY(), -m_controller.getRightX());
    
    //Get a reading from the second sensor, scale it by the voltageScaleFactor, and then scale to Centimeters
    ultrasonicSensorTwoRange = ultrasonicSensorTwo.getValue()*voltageScaleFactor*0.125;
  }

  @Override
  public void disabledInit() {
    //If the robot is disabled, turn off both sensors
    turnOffBothSensors();
  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
