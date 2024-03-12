// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.robotalons.crescendo.commands;

import org.robotalons.crescendo.subsystems.drive.Swerve;
import org.robotalons.crescendo.subsystems.drive.Constants.Measurements;

import com.jcabi.log.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveDriveCommand extends Command {
  /** Creates a new SwerveDriveCommand. */
  private final Swerve swerveSubsystem;
  private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
  private final Supplier<Boolean> fieldOrientedFunction;
  private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

  public SwerveDriveCommand(Swerve swerveSubsystem, Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction,
  Supplier<Boolean> fieldOrientedFunction) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerveSubsystem = swerveSubsystem;
    this.xSpdFunction = xSpdFunction;
    this.ySpdFunction = ySpdFunction;
    this.turningSpdFunction = turningSpdFunction;
    this.fieldOrientedFunction = fieldOrientedFunction;
    this.xLimiter = new SlewRateLimiter(Measurements.ROBOT_MAXIMUM_DRIVE_VELOCITY);
    this.yLimiter = new SlewRateLimiter(Measurements.ROBOT_MAXIMUM_DRIVE_VELOCITY);
    this.turningLimiter = new SlewRateLimiter(Measurements.ROBOT_MAXIMUM_ANGULAR_VELOCITY);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed = xSpdFunction.get();
    double ySpeed = ySpdFunction.get();
    double theta = turningSpdFunction.get();

    xSpeed = xLimiter.calculate(xSpeed);
    ySpeed = yLimiter.calculate(ySpeed);
    theta = turningLimiter.calculate(theta) * Measurements.ROBOT_MAXIMUM_ANGULAR_VELOCITY;

    ChassisSpeeds chassisSpeeds;
      if (fieldOrientedFunction.get()) {
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        xSpeed, ySpeed, theta, swerveSubsystem.getRotation2d());}

      else {
        chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, theta);}


    SwerveModuleState[] moduleStates = Swerve.KINEMATICS.toSwerveModuleStates(chassisSpeeds);

        
    swerveSubsystem.setModuleStates(moduleStates);


  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
