// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Teleop;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.GeneralConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Subsystems.SwerveDrive.SwerveDrivetrain;

public class SwerveDriveTeleop extends Command {
  /** Creates a new SwerveDriveTeleop. */
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");

  private final SwerveDrivetrain swerveDrivetrain;
  private final Supplier<Double> xSpeedFunc, ySpeedFunc, rotationFunc;
  private final Supplier<Boolean> fieldOrientedFunc, limelightAiming;
  private final SlewRateLimiter xRateLimiter, yRateLimiter, rotationRateLimiter;


  public SwerveDriveTeleop(SwerveDrivetrain swerveDrivetrain,Supplier<Double> xSpeedFunc,
  Supplier<Double> ySpeedFunc, Supplier<Double> rotationFunc, Supplier<Boolean> fieldOrientedFunc, Supplier<Boolean> limelightAiming) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerveDrivetrain = swerveDrivetrain;
    this.xSpeedFunc = xSpeedFunc;
    this.ySpeedFunc = ySpeedFunc;
    this.rotationFunc = rotationFunc;
    this.fieldOrientedFunc = fieldOrientedFunc;
    this.limelightAiming = limelightAiming;
    this.xRateLimiter = new SlewRateLimiter(DriveConstants.AccelDriveLimiterUnitPerSec);
    this.yRateLimiter = new SlewRateLimiter(DriveConstants.AccelDriveLimiterUnitPerSec);
    this.rotationRateLimiter = new SlewRateLimiter(DriveConstants.AccelSteerLimiterUnitsPerSec);
    addRequirements(swerveDrivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed = xSpeedFunc.get();
    double ySpeed = ySpeedFunc.get();
    double rotationSpeed = rotationFunc.get();

    xSpeed = Math.abs(xSpeed) > GeneralConstants.Deadzone ? xSpeed : 0.0;
    ySpeed = Math.abs(ySpeed) > GeneralConstants.Deadzone ? ySpeed : 0.0;
    rotationSpeed = Math.abs(rotationSpeed) > GeneralConstants.Deadzone ? rotationSpeed : 0.0;

    xSpeed = xRateLimiter.calculate(xSpeed) * DriveConstants.TeleopMaxSpeedMetersPerSec;
    ySpeed = yRateLimiter.calculate(ySpeed) * DriveConstants.TeleopMaxSpeedMetersPerSec;
    rotationSpeed = rotationRateLimiter.calculate(rotationSpeed) * DriveConstants.TeleopMaxSteerSpeedRadsPerSec;

    ChassisSpeeds chassisSpeeds;

    if (fieldOrientedFunc.get()){
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotationSpeed, swerveDrivetrain.getRotation2d());
    } else if(limelightAiming.get()){
      chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rotationSpeed); 
    }else {
      chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rotationSpeed);
    }

    SwerveModuleState[] moduleStates = DriveConstants.DriveKinematics.toSwerveModuleStates(chassisSpeeds);

    swerveDrivetrain.setModuleStates(moduleStates);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
