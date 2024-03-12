// In Java We Trust

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.robotalons.crescendo;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.robotalons.crescendo.subsystems.drive.Drive;
import org.robotalons.crescendo.commands.*;

public class RobotContainer {
  private Drive robotDrive;

  private CommandXboxController pilotController = new CommandXboxController(0);
  private CommandXboxController copilotController = new CommandXboxController(1);

  private LoggedDashboardChooser<Command> autoChooser;

  public RobotContainer() {
    initializeSubsystems();

    configureAutonomous();

    // AutoBuilder is configured when Drive is initialized, thus chooser must be instantiated after
    // initializeSubsystems()
    autoChooser =
        new LoggedDashboardChooser<>("Autonomous Selector", AutoBuilder.buildAutoChooser());

    configureButtonBindings();
  }

  /** Instantiate subsystems */
  private void initializeSubsystems() {
    switch (Constants.currentMode) {
      case REAL:
        robotDrive =
            new Drive(
                new ModuleIOSparkMax(0),
                new ModuleIOSparkMax(1),
                new ModuleIOSparkMax(2),
                new ModuleIOSparkMax(3),
                new GyroIOPigeon2(false));
        break;
      case SIM:
        robotDrive =
            new Drive(
                new ModuleIOSim(0),
                new ModuleIOSim(1),
                new ModuleIOSim(2),
                new ModuleIOSim(3),
                new GyroIO() {});
        break;
      default:
        robotDrive =
            new Drive(
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new GyroIO() {});
        break;
    }
  }

  /** Register commands with PathPlanner and add default autos to chooser */
  private void configureAutonomous() {
    // Register commands with PathPlanner's AutoBuilder so it can call them
    NamedCommands.registerCommand(
        "Print Pose", Commands.print("Pose: " + robotDrive.getPosition()));
  }

  /** Configure controllers */
  private void configureButtonBindings() {
    /* Pilot bindings */

    /* Drive with joysticks */
    robotDrive.setDefaultCommand(
        SwerveCommands.swerveDrive(
            robotDrive,
            () -> -pilotController.getLeftY(),
            () -> -pilotController.getLeftX(),
            () -> -pilotController.getRightX()));

    /* Reset gyro */
    pilotController.y().onTrue(SwerveCommands.resetGyro(robotDrive));


    /* Move back slightly */
    pilotController
        .povLeft()
        .whileTrue(SwerveCommands.swerveDrive(robotDrive, () -> -0.3, () -> 0.0, () -> 0.0))
        .onFalse(SwerveCommands.stopDrive(robotDrive));

    /* Move forward slightly */
    pilotController
        .povRight()
        .whileTrue(SwerveCommands.swerveDrive(robotDrive, () -> 0.3, () -> 0.0, () -> 0.0))
        .onFalse(SwerveCommands.stopDrive(robotDrive));


  /** Returns the selected autonomous */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}