// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Commands.Teleop.SwerveDriveTeleop;
import frc.robot.Subsystems.SwerveDrive.SwerveDrivetrain;

/** Add your docs here. */
public class RobotContainer {
    private final SwerveDrivetrain swerveDrivetrain = new SwerveDrivetrain();

    private final Joystick DriverStick = new Joystick(0);

    double LimelightAim(){
    double kP = ModuleConstants.kPSteering;
    double TargetAngleVelocity  = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(5) //
                   * kP;
    TargetAngleVelocity *= DriveConstants.TeleopMaxSteerSpeedRadsPerSec;
    TargetAngleVelocity *= -1.0;

    return TargetAngleVelocity;

  }

    public RobotContainer() {
        swerveDrivetrain.setDefaultCommand(new SwerveDriveTeleop(swerveDrivetrain, 
        () -> -DriverStick.getRawAxis(0), 
        () -> DriverStick.getRawAxis(1), 
        () -> DriverStick.getRawAxis(2), 
        () -> !DriverStick.getRawButton(1), 
        null));

        configureButtonBindings();
    }

    private void configureButtonBindings(){
        new JoystickButton(DriverStick, 2).onTrue(Commands.runOnce(swerveDrivetrain::zeroHeading, swerveDrivetrain));
        
        new JoystickButton(DriverStick, 3).whileTrue(new SwerveDriveTeleop(swerveDrivetrain, 
        () -> -DriverStick.getRawAxis(0), 
        () -> DriverStick.getRawAxis(1), 
        () -> LimelightAim(),
        null, 
        () -> !DriverStick.getRawButton(1)));

    }

}
