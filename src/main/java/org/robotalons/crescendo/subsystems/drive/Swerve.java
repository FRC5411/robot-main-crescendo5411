// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.robotalons.crescendo.subsystems.drive;

import org.robotalons.crescendo.subsystems.drive.Constants.Measurements.Modules.*;
import org.robotalons.crescendo.subsystems.drive.Constants.Measurements;
import org.robotalons.crescendo.subsystems.drive.Constants.Ports;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
  private static SwerveModule FL_MODULE;
  private static SwerveModule FR_MODULE;
  private static SwerveModule RL_MODULE;
  private static SwerveModule RR_MODULE;
  private static Pose2d INITAL;
  private static Pigeon2 GYRO;
  public static SwerveDriveKinematics KINEMATICS;
  private static SwerveDriveKinematicsConstraint CONSTRAINTS;
  private static SwerveDrivePoseEstimator POSE_ESTIMATOR;
  private static SwerveModule[] MODULES;

  /** Creates a new Swerve. */
  public Swerve() {
    new Thread(() -> {
      try {
        Thread.sleep(1000);
          zeroHeading();
        } catch (Exception e) {
      }
      }).start();

    FL_MODULE = new SwerveModule(
      (FL.DRIVE_CONTROLLER_ID),
      (FL.ASIMUTH_CONTROLLER_ID),
      (FL.DRIVE_INVERTED),
      (FL.ASIMUTH_INVERTED),
      (FL.NUMBER),
      (FL.ASIMUTH_ENCODER_OFFSET),
      (FL.ABSOLUTE_ENCODER_ID),
      (true)
    );

    FR_MODULE = new SwerveModule(
      (FR.DRIVE_CONTROLLER_ID),
      (FR.ASIMUTH_CONTROLLER_ID),
      (FR.DRIVE_INVERTED),
      (FR.ASIMUTH_INVERTED),
      (FR.NUMBER),
      (FR.ASIMUTH_ENCODER_OFFSET),
      (FR.ABSOLUTE_ENCODER_ID),
      (true)
    );

    RL_MODULE = new SwerveModule(
      (RL.DRIVE_CONTROLLER_ID),
      (RL.ASIMUTH_CONTROLLER_ID),
      (RL.DRIVE_INVERTED),
      (RL.ASIMUTH_INVERTED),
      (RL.NUMBER),
      (RL.ASIMUTH_ENCODER_OFFSET),
      (RL.ABSOLUTE_ENCODER_ID),
      (true)
    );

    RR_MODULE = new SwerveModule(
      (RR.DRIVE_CONTROLLER_ID),
      (RR.ASIMUTH_CONTROLLER_ID),
      (RR.DRIVE_INVERTED),
      (RR.ASIMUTH_INVERTED),
      (RR.NUMBER),
      (RR.ASIMUTH_ENCODER_OFFSET),
      (RR.ABSOLUTE_ENCODER_ID),
      (true)
    );

    GYRO = new Pigeon2(Ports.GYROSCOPE_ID);

    KINEMATICS =  new SwerveDriveKinematics(
      new Translation2d((0.3639), (0.3579)),
      new Translation2d((0.3639), (-0.3579)),
      new Translation2d((-0.3639), (0.3579)),
      new Translation2d((-0.3639), (-0.3579))
      );
    
    INITAL = new Pose2d();

    CONSTRAINTS = new SwerveDriveKinematicsConstraint((KINEMATICS), (Measurements.ROBOT_MAXIMUM_DRIVE_VELOCITY ));

    POSE_ESTIMATOR = new SwerveDrivePoseEstimator(KINEMATICS, getRotation2D(), getModulePositions(), INITAL);

    MODULES = new SwerveModule[]{FL_MODULE, FR_MODULE, RL_MODULE, RR_MODULE};
  }

  public double getYaw(){
    return GYRO.getYaw().getValueAsDouble();
  }

  public double getPitch(){
    return GYRO.getPitch().getValueAsDouble();
  }

  public double getRoll(){
    return GYRO.getRoll().getValueAsDouble();
  }

  public Rotation2d getRotation2D(){
    return GYRO.getRotation2d();
  }

  public SwerveModulePosition[] getModulePositions() {
      SwerveModulePosition[] positions = new SwerveModulePosition[4];
      for (int i = 0; i < 4; i++) {
        positions[i] =
            MODULES[i] != null ? MODULES[i].getModulePosition() : new SwerveModulePosition();
      }

      return positions;
    }

  public void stopModules() {
    FL_MODULE.cease();
    FR_MODULE.cease();
    RL_MODULE.cease();
    RR_MODULE.cease();
  }

  public double getHeading() {
    return Math.IEEEremainder(GYRO.getAngle(), 360);
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getHeading());
  }

  public void zeroHeading() {
    resetPosistion(getRotation2d());
  }

  public void resetPosistion(Rotation2d rot){
    GYRO.setYaw(rot.getDegrees());
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Measurements.ROBOT_MAXIMUM_DRIVE_VELOCITY);
    FL_MODULE.setDesiredState(desiredStates[0]);
    FR_MODULE.setDesiredState(desiredStates[1]);
    RL_MODULE.setDesiredState(desiredStates[2]);
    RR_MODULE.setDesiredState(desiredStates[3]);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
