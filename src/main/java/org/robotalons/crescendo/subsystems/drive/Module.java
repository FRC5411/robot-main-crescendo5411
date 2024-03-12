// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.robotalons.crescendo.subsystems.drive;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class Module {
  private static final String[] moduleNames = new String[] {"FL", "FR", "BL", "BR"};

  private final int index;
  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private SimpleMotorFeedforward ff =
      new SimpleMotorFeedforward(DriveConstants.moduleConstants.ffkS(), DriveConstants.moduleConstants.ffkV(), 0.0);
  @Getter private SwerveModuleState setpointState = new SwerveModuleState();
  

  public Module(ModuleIO io, int index) {
    this.io = io;
    this.index = index;
  }

  /** Called while blocking odometry thread */
  public void updateInputs() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive/Module" + index, inputs);

  }

  /** Runs to {@link SwerveModuleState} */
  public void runSetpoint(SwerveModuleState setpoint, SwerveModuleState torqueFF) {
    setpointState = setpoint;
    double wheelTorqueNm =
        torqueFF.speedMetersPerSecond; // Using SwerveModuleState for torque for easy logging
    io.runDriveVelocitySetpoint(
        setpoint.speedMetersPerSecond / DriveConstants.driveConfig.wheelRadius(),
        ff.calculate(setpoint.speedMetersPerSecond / DriveConstants.driveConfig.wheelRadius())
            + ((wheelTorqueNm / DriveConstants.moduleConstants.driveReduction()) * DriveConstants.moduleConstants.ffkT()));
    io.runTurnPositionSetpoint(setpoint.angle.getRadians());
  }

  /** Runs characterization volts or amps depending on using voltage or current control. */
  public void runCharacterization(double turnSetpointRads, double input) {
    io.runTurnPositionSetpoint(turnSetpointRads);
    io.runCharacterization(input);
  }

  /** Stops motors. */
  public void stop() {
    io.stop();
  }

  /** Get all latest {@link SwerveModulePosition}'s from last cycle. */
  public SwerveModulePosition[] getModulePositions() {
    int minOdometryPositions =
        Math.min(inputs.odometryDrivePositionsMeters.length, inputs.odometryTurnPositions.length);
    SwerveModulePosition[] positions = new SwerveModulePosition[minOdometryPositions];
    for (int i = 0; i < minOdometryPositions; i++) {
      positions[i] =
          new SwerveModulePosition(
              inputs.odometryDrivePositionsMeters[i], inputs.odometryTurnPositions[i]);
    }
    return positions;
  }

  /** Get turn angle of module as {@link Rotation2d}. */
  public Rotation2d getAngle() {
    return inputs.turnAbsolutePosition;
  }

  /** Get position of wheel rotations in radians */
  public double getPositionRads() {
    return inputs.drivePositionRads;
  }

  /** Get position of wheel in meters. */
  public double getPositionMeters() {
    return inputs.drivePositionRads * DriveConstants.driveConfig.wheelRadius();
  }

  /** Get velocity of wheel in m/s. */
  public double getVelocityMetersPerSec() {
    return inputs.driveVelocityRadsPerSec * DriveConstants.driveConfig.wheelRadius();
  }

  /** Get current {@link SwerveModulePosition} of module. */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  /** Get current {@link SwerveModuleState} of module. */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }

  /** Get velocity of drive wheel for characterization */
  public double getCharacterizationVelocity() {
    return inputs.driveVelocityRadsPerSec;
  }
}
