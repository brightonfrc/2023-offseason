// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.RunIntakeCommand;
import frc.robot.subsystems.DriveSubSystem;
import frc.robot.subsystems.IntakeSubsystem;

import javax.xml.namespace.QName;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private DriveSubSystem DRIVE_SUBSYSTEM = new DriveSubSystem();
  private IntakeSubsystem INTAKE_SUBSYSTEM = new IntakeSubsystem();
  private Joystick controller = new Joystick(0);
  private DefaultDriveCommand defaultDriveCommand = new DefaultDriveCommand(DRIVE_SUBSYSTEM, controller);
  private JoystickButton A_BUTTON = new JoystickButton(controller, 0);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    defaultCommands();
  }

  private void configureBindings() {
    A_BUTTON.toggleOnTrue(new RunIntakeCommand(INTAKE_SUBSYSTEM));
  }

  private void defaultCommands() {
    DRIVE_SUBSYSTEM.setDefaultCommand(new DefaultDriveCommand(DRIVE_SUBSYSTEM, controller));
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new SequentialCommandGroup(null);
  }
}
