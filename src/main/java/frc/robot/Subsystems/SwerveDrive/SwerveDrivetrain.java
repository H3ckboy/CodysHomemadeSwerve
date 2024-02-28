// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.SwerveDrive;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.GeneralConstants;

public class SwerveDrivetrain extends SubsystemBase {

    private final SwerveModule frontLeft = new SwerveModule(
      DriveConstants.FLDriveMotorID, 
      DriveConstants.FLSteerMotorID, 
      DriveConstants.FLDriveEncoderRev, 
      DriveConstants.FLSteerEncoderRev, 
      DriveConstants.FLAbsoluteEncoderID, 
      DriveConstants.FLAbsoluteEncoderOffRads, 
      DriveConstants.FLAbsoluteEncoderRev);
      
    private final SwerveModule frontRight = new SwerveModule(
      DriveConstants.FRDriveMotorID, 
      DriveConstants.FRSteerMotorID, 
      DriveConstants.FRDriveEncoderRev, 
      DriveConstants.FRSteerEncoderRev, 
      DriveConstants.FRAbsoluteEncoderID, 
      DriveConstants.FRAbsoluteEncoderOffRads, 
      DriveConstants.FRAbsoluteEncoderRev);

    private final SwerveModule backLeft = new SwerveModule(
      DriveConstants.BLDriveMotorID, 
      DriveConstants.BLSteerMotorID, 
      DriveConstants.BLDriveEncoderRev, 
      DriveConstants.BLSteerEncoderRev, 
      DriveConstants.BLAbsoluteEncoderID, 
      DriveConstants.BLAbsoluteEncoderOffRads, 
      DriveConstants.BLAbsoluteEncoderRev);

    private final SwerveModule backRight = new SwerveModule(
      DriveConstants.BRDriveMotorID, 
      DriveConstants.BRSteerMotorID, 
      DriveConstants.BRDriveEncoderRev, 
      DriveConstants.BRSteerEncoderRev, 
      DriveConstants.BRAbsoluteEncoderID, 
      DriveConstants.BRAbsoluteEncoderOffRads, 
      DriveConstants.BRAbsoluteEncoderRev);

  private final WPI_PigeonIMU pigeon = new WPI_PigeonIMU(GeneralConstants.pigeonID);


  /** Creates a new SwerveDrivetrain. */
  public SwerveDrivetrain() {
    pigeon.configFactoryDefault();

    new Thread(() -> {
      try {
          Thread.sleep(1000);
          zeroGyro();
      } catch (Exception e) {
      }
  }).start();

  }

  public void zeroGyro() {
    pigeon.setYaw(0);
  }

  public void zeroHeading(){
    pigeon.reset();
  }

  public double getHeading(){
    return Math.IEEEremainder(pigeon.getAngle(), 360);
  }

  public Rotation2d getRotation2d(){
    return Rotation2d.fromDegrees(getHeading());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Robot Heading", getHeading());
  }

  public void stopModules() {
    frontLeft.stopMotors();
    frontRight.stopMotors();
    backLeft.stopMotors();
    backRight.stopMotors();
}

  public void setModuleStates(SwerveModuleState[] desiredState){
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredState, DriveConstants.PhysicalMetersPerSecMax);
    frontLeft.setModuleState(desiredState[0]);
    frontRight.setModuleState(desiredState[1]);
    backLeft.setModuleState(desiredState[2]);
    backRight.setModuleState(desiredState[3]);
  }
}
