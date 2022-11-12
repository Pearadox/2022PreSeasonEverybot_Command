// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class DriveTrain extends SubsystemBase {
  
  private CANSparkMax _frontLeft = new CANSparkMax(11, MotorType.kBrushless);
  private CANSparkMax _backLeft = new CANSparkMax(10, MotorType.kBrushless);
  private CANSparkMax _frontRight = new CANSparkMax(12, MotorType.kBrushless);
  private CANSparkMax _backRight = new CANSparkMax(13, MotorType.kBrushless);
  private DifferentialDrive drive = new DifferentialDrive(_frontLeft, _frontRight);

  /** Creates a new DriveTrain. */
  public DriveTrain() {
    SendableRegistry.add(drive, "drive");

    _frontLeft.restoreFactoryDefaults();
    _frontLeft.burnFlash();
    _backLeft.restoreFactoryDefaults();
    _backLeft.burnFlash();
    _frontRight.restoreFactoryDefaults();
    _frontRight.burnFlash();
    _backRight.restoreFactoryDefaults();
    _backRight.burnFlash();

    _backLeft.follow(_frontLeft);
    _backRight.follow(_frontRight);
  }

  @Override
  public void periodic() {}

  public void teleopDrive(Joystick controller) {
    drive.arcadeDrive(controller.getRawAxis(4), controller.getRawAxis(1));
  }
}
