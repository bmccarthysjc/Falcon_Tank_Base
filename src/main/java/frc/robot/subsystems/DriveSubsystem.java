// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.*;

import static java.lang.Math.IEEEremainder;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SPI;


public class DriveSubsystem extends SubsystemBase {
  
  private final WPI_TalonFX leftMaster = new WPI_TalonFX(CanIdConstants.LEFT_MASTER_ID);
  private final WPI_TalonFX rightMaster = new WPI_TalonFX(CanIdConstants.RIGHT_MASTER_ID);
  private final WPI_TalonSRX leftFollower = new WPI_TalonSRX(CanIdConstants.LEFT_FOLLOWER_ID);
  private final WPI_TalonSRX rightFollower = new WPI_TalonSRX(CanIdConstants.RIGHT_FOLLOWER_ID);
  
  private final DifferentialDrive differentialDrive = new DifferentialDrive(leftMaster, rightMaster);
  private final SupplyCurrentLimitConfiguration currentLimit = new SupplyCurrentLimitConfiguration(true, 40, 60, 1);

  private final ADXRS450_Gyro gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);

  public DriveSubsystem() {
    rightFollower.follow(rightMaster);
    leftFollower.follow(leftMaster);

    leftMaster.configSupplyCurrentLimit(currentLimit);
    rightMaster.configSupplyCurrentLimit(currentLimit);
    leftFollower.configSupplyCurrentLimit(currentLimit);
    rightFollower.configSupplyCurrentLimit(currentLimit);

    leftMaster.setNeutralMode(NeutralMode.Brake);
    rightMaster.setNeutralMode(NeutralMode.Brake);
    leftFollower.setNeutralMode(NeutralMode.Brake);
    rightFollower.setNeutralMode(NeutralMode.Brake);

    leftMaster.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);
    rightMaster.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);

    leftMaster.setInverted(TalonFXInvertType.Clockwise);
    rightMaster.setInverted(TalonFXInvertType.CounterClockwise);
    leftFollower.setInverted(InvertType.FollowMaster);
    rightFollower.setInverted(InvertType.FollowMaster);

    resetEncoders();
    resetGyro();
  }

  public void arcadeDrive(double fwd, double rot){
    differentialDrive.arcadeDrive(fwd, rot);
}

public void gyroAngle(){
  gyro.getAngle();
}

public double getHeading(){
  return IEEEremainder(gyro.getAngle(), 360);
}

public void calibrateGyro(){
  gyro.calibrate();
}

public void resetGyro(){
  gyro.reset();
}

public void resetEncoders(){
  leftMaster.setSelectedSensorPosition(0, 0, 10);
  rightMaster.setSelectedSensorPosition(0, 0, 10);
}

public double getAverageDistance(){
  return ((getLeftWheelPosition() + getRightWheelPosition())/2);
}

public DifferentialDriveWheelSpeeds getWheelSpeeds(){
  return new DifferentialDriveWheelSpeeds(getLeftWheelSpeed(), getRightWheelSpeed());

}

private double getLeftWheelPosition(){
  return (leftMaster.getSelectedSensorPosition() *DriveConstants.WHEEL_CIRCUMFERENCE_METERS / DriveConstants.TALONFX_ENCODER_CPR)
   / DriveConstants.GEAR_RATIO;
}
private double getRightWheelPosition(){
  return (rightMaster.getSelectedSensorPosition() *DriveConstants.WHEEL_CIRCUMFERENCE_METERS / DriveConstants.TALONFX_ENCODER_CPR)
   / DriveConstants.GEAR_RATIO;
}

private double getLeftWheelSpeed(){
  return leftMaster.getSelectedSensorVelocity(0) * 10/ DriveConstants.TALONFX_ENCODER_CPR /DriveConstants.GEAR_RATIO
  *DriveConstants.WHEEL_CIRCUMFERENCE_METERS;
}

private double getRightWheelSpeed(){
  return leftMaster.getSelectedSensorVelocity(0) * 10/ DriveConstants.TALONFX_ENCODER_CPR /DriveConstants.GEAR_RATIO
  *DriveConstants.WHEEL_CIRCUMFERENCE_METERS;
}

  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
