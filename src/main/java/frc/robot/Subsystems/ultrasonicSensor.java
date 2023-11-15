package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AnalogInput;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class UltrasonicSensor extends SubsystemBase {
  public DigitalOutput ultrasonicTriggerPinOne = new DigitalOutput(0);
  public DigitalOutput ultrasonicTriggerPinTwo = new DigitalOutput(1);
 
  public AnalogInput ultrasonicSensorOne = new AnalogInput(0);
  public AnalogInput ultrasonicSensorTwo = new AnalogInput(1);
 
  public double ultrasonicSensorOneRange = 0;
  public double ultrasonicSensorTwoRange = 0;
 
  public double voltageScaleFactor = 1;
 
  public void turnOnUltrasonicSensorOne() {
    ultrasonicTriggerPinOne.set(true);
    ultrasonicTriggerPinTwo.set(false);
  }
 
  public void turnOnUltrasonicSensorTwo() {
    ultrasonicTriggerPinOne.set(false);
    ultrasonicTriggerPinTwo.set(true);
  }
 
  public void turnOffBothUltrasonicSensors() {
    ultrasonicTriggerPinOne.set(false);
    ultrasonicTriggerPinTwo.set(false);
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public void init() {
    SmartDashboard.putNumber("Ultrasonic Sensor 1 Range", 500);
    SmartDashboard.putNumber("Ultrasonic Sensor 2 Range", 500);
  }

  public void periodic() {
    //Publish range readings to SmartDashboard
    SmartDashboard.putNumber("Ultrasonic Sensor 1 Range", ultrasonicSensorOneRange);
    SmartDashboard.putNumber("Ultrasonic Sensor 2 Range", ultrasonicSensorTwoRange);
 
    voltageScaleFactor = 5/RobotController.getVoltage5V(); //Calculate what percentage of 5 Volts we are actually at
  }

  /** This function is run once each time the robot enters autonomous mode. */
  public void autonomousInit() {
    //If we are in Autonomous mode, turn on the first sensor (and turn off the second sensor)
    turnOnUltrasonicSensorOne();
  }

  /** This function is called periodically during autonomous. */
  public void autonomousPeriodic() {
    //Get a reading from the first sensor, scale it by the voltageScaleFactor, and then scale to Centimeters
    ultrasonicSensorOneRange = ultrasonicSensorOne.getValue()*voltageScaleFactor*0.125;
  }

  
  //This function is called once each time the robot enters teleoperated mode. 

  public void teleopInit() {
    //If we are in Teleoperated mode, turn on the second sensor (and turn off the first sensor)
    turnOnUltrasonicSensorTwo();
  }

  /** This function is called periodically during teleoperated mode. */
  public void teleopPeriodic() {
    //Get a reading from the second sensor, scale it by the voltageScaleFactor, and then scale to Centimeters
    ultrasonicSensorTwoRange = ultrasonicSensorTwo.getValue()*voltageScaleFactor*0.125;
  }

  public void disabledInit() {
    //If the robot is disabled, turn off both sensors
    turnOffBothUltrasonicSensors();
  }
}