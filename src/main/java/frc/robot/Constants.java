// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class Constants {
    public final class GeneralConstants{
        public static final double Deadzone = 0.075;

        public static final MotorType NeoMotor = MotorType.kBrushless;
        
        public static final int pigeonID = 13;
        public static final boolean invertGyro = false;
    }

    public static final class ModuleConstants{
        public static final double WheelDiameterMeters = Units.inchesToMeters(4);
        public static final double DriveMotorGearRatio = (6.75 / 1);
        public static final double SteerMotorGearRatio = ((150/7) / 1);
        public static final double DriveEncoderRot2Meter = (DriveMotorGearRatio * Math.PI * WheelDiameterMeters);
        public static final double SteerEncoderRot2Rad = (SteerMotorGearRatio * 2 * Math.PI);
        public static final double SteerEncoderRad2Deg = (SteerEncoderRot2Rad * (180/Math.PI));
        public static final double DriveEncoderRPM2MeterPerSec = (DriveEncoderRot2Meter / 60);
        public static final double SteerEncoderRPM2RadsPerSec = (SteerEncoderRot2Rad / 60);
        public static final double SteerEncoderRPM2DegPerSec = (SteerEncoderRad2Deg / 60); 
        public static final double kPSteering = 0.5;
    }

    public static final class DriveConstants{
        
        public static final double RobotWidth = Units.inchesToMeters(28.25);
        public static final double RobotLength = Units.inchesToMeters(28.25);

        public static final SwerveDriveKinematics DriveKinematics = new SwerveDriveKinematics(
            new Translation2d(RobotLength, RobotWidth), //FL
            new Translation2d(RobotLength, -RobotWidth), //FR
            new Translation2d(-RobotLength, RobotWidth), //BL
            new Translation2d(-RobotLength, -RobotWidth) //BR
        );

        public static final int FLDriveMotorID = 1;
        public static final int FRDriveMotorID = 4;
        public static final int BLDriveMotorID = 7;
        public static final int BRDriveMotorID = 10;

        public static final int FLSteerMotorID = 2;
        public static final int FRSteerMotorID = 5;
        public static final int BLSteerMotorID = 8;
        public static final int BRSteerMotorID = 11;

        public static final boolean FLDriveEncoderRev = true;
        public static final boolean FRDriveEncoderRev = true;
        public static final boolean BLDriveEncoderRev = true;
        public static final boolean BRDriveEncoderRev = true;

        public static final boolean FLSteerEncoderRev = true;
        public static final boolean FRSteerEncoderRev = true;
        public static final boolean BLSteerEncoderRev = true;
        public static final boolean BRSteerEncoderRev = true;

        public static final int FLAbsoluteEncoderID = 3;
        public static final int FRAbsoluteEncoderID = 6;
        public static final int BLAbsoluteEncoderID = 9;
        public static final int BRAbsoluteEncoderID = 12;

        public static final boolean FLAbsoluteEncoderRev = true;
        public static final boolean FRAbsoluteEncoderRev = true;
        public static final boolean BLAbsoluteEncoderRev = true;
        public static final boolean BRAbsoluteEncoderRev = true;

        public static final double FLAbsoluteEncoderOffRads = 1.3;
        public static final double FRAbsoluteEncoderOffRads = -3;
        public static final double BLAbsoluteEncoderOffRads = -1.1;
        public static final double BRAbsoluteEncoderOffRads = -3.5;

        public static final double PhysicalMetersPerSecMax = 4.602;
        public static final double PhysicalAngleSpeedRadsPerSec = 2 * 2 * Math.PI;
        public static final double PhysicalAngleSpeedDegPerSec = 2 * (180/Math.PI);

        public static final double TeleopMaxSpeedMetersPerSec = PhysicalMetersPerSecMax / 4;
        public static final double TeleopMaxSteerSpeedRadsPerSec = PhysicalAngleSpeedRadsPerSec / 4;
        public static final double TeleopMaxSteerSpeedDegPerSec = PhysicalAngleSpeedDegPerSec / 4;
        public static final double AccelDriveLimiterUnitPerSec = 3;
        public static final double AccelSteerLimiterUnitsPerSec = 3;


    }
}
