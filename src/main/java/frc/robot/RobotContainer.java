// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.time.Instant;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import frc.robot.commands.*;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  private final Joystick _controller = new Joystick(0);
  private final ADIS16470_IMU _gyro = new ADIS16470_IMU();
  private final DriveTrain _driveTrain = new DriveTrain(_gyro);
  private final Intake _intake = new Intake();
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    _gyro.reset();
    _gyro.setYawAxis(IMUAxis.kY);
    configureButtonBindings();
    _driveTrain.setDefaultCommand(new ArcadeDrive(_driveTrain, _controller));
    _intake.setDefaultCommand(new IntakeStop(_intake));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(_controller, 6).whileHeld(new IntakeIn(_intake));
    new JoystickButton(_controller, 5).whileHeld(new IntakeOut(_intake));
    new JoystickButton(_controller, 4).whileHeld(new RunCommand(_intake::raise, _intake));
    new JoystickButton(_controller, 1).whileHeld(new RunCommand(_intake::lower, _intake));
    new JoystickButton(_controller, 2).whileHeld(new Stabilize(_driveTrain, _gyro));
    new JoystickButton(_controller, 3).whenPressed(new InstantCommand(_gyro::reset));
    new JoystickButton(_controller, 8).whenPressed(new EngageCS(_driveTrain, _gyro));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
