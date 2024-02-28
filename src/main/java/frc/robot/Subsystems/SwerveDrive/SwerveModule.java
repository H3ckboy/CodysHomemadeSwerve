// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.SwerveDrive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.sensors.CANCoder;

import frc.robot.Constants.GeneralConstants;
import frc.robot.Constants.ModuleConstants;

/** Add your docs here. */
public class SwerveModule {
    private final CANSparkMax driveMotor;
    private final CANSparkMax steerMotor;

    private final PIDController steeringPIDController;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder steeringEncoder;

    private final CANCoder absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRads;


    //Creating a new swerve Module with specifications
    //for the MotorID, isReversed, absEncoder, absOffset, and absIsReversed
    public SwerveModule(int driveMotorID, int steeringMotorID, boolean driveMotorReversed,
    boolean steeringMotorReversed, int absoluteEncoderID, double absoluteEncoderOffset,
    boolean absoluteEncoderReversed){

        //CANCoder works in degrees, with this we will provide the predetermined offset
        //check to see if the CANCoder is working in a reversed state
        //and creates a CANCoder instance based on the provided ID
        this.absoluteEncoderOffsetRads = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new CANCoder(absoluteEncoderID);

        //will create NeoMotors for every motor instance along with the assigned MotorID
        driveMotor = new CANSparkMax(driveMotorID, GeneralConstants.NeoMotor);
        steerMotor = new CANSparkMax(steeringMotorID, GeneralConstants.NeoMotor);

        //determines whether a motor will be reversed or not
        driveMotor.setInverted(driveMotorReversed);
        steerMotor.setInverted(steeringMotorReversed);

        //getting the encoders from the respective motor
        driveEncoder = driveMotor.getEncoder();
        steeringEncoder = steerMotor.getEncoder();

        //driveEncoder Converting from Rotations to meters
        driveEncoder.setPositionConversionFactor(ModuleConstants.DriveEncoderRot2Meter);
        driveEncoder.setVelocityConversionFactor(ModuleConstants.DriveEncoderRPM2MeterPerSec);
        //steeringEncoder converting from rotations to degrees (in a messy way)
        //steeringEncoder.setPositionConversionFactor(ModuleConstants.SteerEncoderRad2Deg);
        //steeringEncoder.setVelocityConversionFactor(ModuleConstants.SteerEncoderRPM2DegPerSec);
        //steeringEncoder converting from rotations to radians (in the event that the degree code doesn't work)
        steeringEncoder.setPositionConversionFactor(ModuleConstants.SteerEncoderRot2Rad);
        steeringEncoder.setVelocityConversionFactor(ModuleConstants.SteerEncoderRPM2RadsPerSec);

        //Creating PIDController for steering motor to use so that it can move smoothlY
        //Also creating a continuous input so that the motor can know its in a circle, can doesn't have to jump
        //around when swiching directions
        steeringPIDController = new PIDController(ModuleConstants.kPSteering, 0, 0);
        steeringPIDController.enableContinuousInput(-Math.PI, Math.PI);

        //run reset encoder function
        resetEncoders();
    }

    //gets relative encoder position from DriveMotor
    public double getDrivePosition(){
        return driveEncoder.getPosition();
    }

    //gets velocity value from DriveMotor
    public double getDriveVelocity(){
        return driveEncoder.getVelocity();
    }

    //gets relative encoder position from SteerMotor
    public double getSteerPosition(){
        return steeringEncoder.getPosition();
    }

    //gets velocity from SteerMotor
    public double getSteerVelocity(){
        return steeringEncoder.getVelocity();
    }

    //gets absolute encoder position and subtracts the offset,
    //then if encoder is reversed, multiplies it by -1 to turn it positive
    public double getAbsoluteEncoder(){
        double angle = absoluteEncoder.getAbsolutePosition();
        angle *= 2 * Math.PI;
        angle -= absoluteEncoderOffsetRads;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    //resets driveEncoder to 0, and sets steeringEncoder to the current Absolute Encoder reading
    public void resetEncoders(){
        driveEncoder.setPosition(0);
        steeringEncoder.setPosition(getAbsoluteEncoder());
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getSteerPosition()));
    }

    public void setModuleState(SwerveModuleState desiredState){
            if(Math.abs(desiredState.speedMetersPerSecond) < 0.001){
                stopMotors();
                return;
            }

        desiredState = SwerveModuleState.optimize(desiredState, getState().angle);
        driveMotor.set(desiredState.speedMetersPerSecond / Units.feetToMeters(15.1));
        steerMotor.set(steeringPIDController.calculate(getSteerPosition(), desiredState.angle.getRadians()));
        SmartDashboard.putString("Swerve [" + absoluteEncoder.getDeviceID() + "] state", desiredState.toString());
    }

    public void stopMotors(){
        driveMotor.set(0);
        steerMotor.set(0);
    }


}
