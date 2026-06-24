// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;

import javax.crypto.spec.DHGenParameterSpec;

import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Ports.DrivetrainPorts;
import frc.robot.subsystems.DriveConstants.Drivetrain;
import frc.robot.subsystems.DriveConstants.Translation;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;

@Logged
public class Drive extends SubsystemBase {
  private final PIDController rotPidController;
  private final PIDController xPidController;
  private final PIDController yPidController;

  private final SwerveDrivePoseEstimator swerveEstimator; // robot position tracker, combines gyro + wheel encoders + vision

  private final ModuleKraken frontLeft;
  private final ModuleKraken frontRight;
  private final ModuleKraken rearLeft;
  private final ModuleKraken rearRight;

  private DoubleSupplier xSpeedRatio; // doublesupplier is a function that returns a double
  private DoubleSupplier ySpeedRatio;
  private DoubleSupplier rotSpeedRatio;

  private final List<ModuleKraken> modules;

  private final Pigeon2 gyro;

  private final SwerveDriveOdometry odometry;

  private ChassisSpeeds speeds = new ChassisSpeeds(); // stores desired robot movement
    
  public Drive() {
    rotPidController = new PidController(DriveConstants.Translation.rotPID.P, DriveConstants.Translation.rotPID.I, DriveConstants.Translation.rotPID.D);
    xPidController = new PidController(DriveConstants.Translation.xPID.P, DriveConstants.Translation.xPID.I, DriveConstants.Translation.xPID.D);
    yPidController = new PidController(DriveConstants.Translation.yPID.P, DriveConstants.Translation.yPID.I, DriveConstants.Translation.yPID.D);

    rotPidController.enableContinuousInput(-Math.PI, Math.PI);
    xPidController.enableContinuousInput(-180, 180);
    yPidController.enableContinuousInput(-180, 180);

    rotPidController.setTolerance(3);
    xPidController.setTolerance(0.05);
    yPidController.setTolerance(0.05);
    
    frontLeft = new ModuleKraken(DrivetrainPorts.FRONT_LEFT_DRIVE, DrivetrainPorts.FRONT_LEFT_TURN, DrivetrainPorts.FRONT_LEFT_CANCODER, Translation.FRONT_LEFT_MAG_OFFSET, Translation.FRONT_LEFT_ANGOFFSET, false);
    frontRight = new ModuleKraken(DrivetrainPorts.FRONT_RIGHT_DRIVE, DrivetrainPorts.FRONT_RIGHT_TURN, DrivetrainPorts.FRONT_RIGHT_CANCODER, Translation.FRONT_RIGHT_MAG_OFFSET, Translation.FRONT_RIGHT_ANGOFFSET, false);
    rearLeft = new ModuleKraken(DrivetrainPorts.REAR_LEFT_DRIVE, DrivetrainPorts.REAR_LEFT_TURN, DrivetrainPorts.REAR_LEFT_CANCODER, Translation.REAR_LEFT_MAG_OFFSET, Translation.REAR_LEFT_ANGOFFSET, false);
    rearRight = new ModuleKraken(DrivetrainPorts.REAR_RIGHT_DRIVE, DrivetrainPorts.REAR_RIGHT_TURN, DrivetrainPorts.REAR_RIGHT_CANCODER, Translation.REAR_RIGHT_MAG_OFFSET, Translation.REAR_RIGHT_ANGOFFSET, false);
    
    modules = List.of(
        frontLeft,
        frontRight,
        rearLeft,
        rearRight
    );

    gyro = new Pigeon2(DrivetrainPorts.GYRO_ID, Translation.CAN_BUS);


  }

  public void driveRatio(DoubleSupplier xSpeedRatio, DoubleSupplier ySpeedRatio, DoubleSupplier rotSpeedRatio) {
    this.xSpeedRatio = xSpeedRatio;
    this.ySpeedRatio = ySpeedRatio;
    this.rotSpeedRatio = rotSpeedRatio;
    double xVel = xSpeedRatio.getAsDouble() * Drivetrain.MAX_SPEED * Drivetrain.SPEED_FACTOR;
    double yVel = ySpeedRatio.getAsDouble() * Drivetrain.MAX_SPEED * Drivetrain.SPEED_FACTOR;
    double rotVel = rotSpeedRatio.getAsDouble() * Drivetrain.MAX_SPEED * Drivetrain.SPEED_FACTOR;

    speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xVel, yVel, rotVel, gyro.getRotation2d());
    SwerveModuleState[] moduleStates = Drivetrain.kDriveKinematics.toSwerveModuleStates(speeds);

    setModuleStates(moduleStates);
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Drivetrain.MAX_SPEED);
    frontLeft.setDesiredStateNoPID(desiredStates[0]);
    frontRight.setDesiredStateNoPID(desiredStates[1]);
    rearLeft.setDesiredStateNoPID(desiredStates[2]);
    rearRight.setDesiredStateNoPID(desiredStates[3]);
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}