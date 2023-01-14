// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class DriveTrain extends SubsystemBase {
  
  private CANSparkMax _frontLeft = new CANSparkMax(11, MotorType.kBrushless);
  private CANSparkMax _backLeft = new CANSparkMax(10, MotorType.kBrushless);
  private CANSparkMax _frontRight = new CANSparkMax(12, MotorType.kBrushless);
  private CANSparkMax _backRight = new CANSparkMax(13, MotorType.kBrushless);
  private DifferentialDrive drive = new DifferentialDrive(_frontLeft, _frontRight);
  private ADIS16470_IMU _gyro;
  
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

  public DriveTrain(ADIS16470_IMU gyro) {
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
    _gyro = gyro;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Gyro Pitch", _gyro.getAngle());
  }

  public void teleopDrive(Joystick controller) {
    double axis4 = controller.getRawAxis(4);
    double axis1 = controller.getRawAxis(1);
    drive(axis4, axis1);
  }

  public void drive(double rotation, double direction){
    drive.arcadeDrive(rotation, direction);
  }

  public RelativeEncoder getEncoder() {
    return _frontLeft.getEncoder();
  }
}
