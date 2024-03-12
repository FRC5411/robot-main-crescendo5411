// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.robotalons.crescendo;


import org.robotalons.crescendo.commands.DriverIntakeFeedback;
import org.robotalons.crescendo.subsystems.cannon.CannonSubsystem;
import org.robotalons.crescendo.subsystems.indexer.Indexer;
import org.robotalons.crescendo.subsystems.intake.Intake;
import org.robotalons.crescendo.subsystems.led.LEDStrip;
import org.robotalons.crescendo.subsystems.led.LEDSubsystem;
import org.robotalons.crescendo.subsystems.led.LEDStrip.LEDIdentifier;
import org.robotalons.lib.controller.AxisButton;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    private final LEDStrip LEDS = new LEDStrip(26, 9);

    private final Joystick driver = new Joystick(0);
    private final Joystick operator = new Joystick(1);
    private final Joystick atari = new Joystick(2);
    // The robot's subsystems and commands are defined here...
    private final JoystickButton rotateWithTag = new JoystickButton(driver, XboxController.Button.kRightStick.value);
    //private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton driveA = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton driveY = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton driveB = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton driveX = new JoystickButton(driver, XboxController.Button.kX.value);
     private final JoystickButton driveLeftBumper = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton driveRightBumper = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    private final JoystickButton driveStart = new JoystickButton(driver, XboxController.Button.kStart.value);
    private final JoystickButton driveSelect = new JoystickButton(driver, XboxController.Button.kBack.value);
    private final AxisButton driveLeftTrigger = new AxisButton(driver, 2, 0.5);
    private final AxisButton driveRightTrigger = new AxisButton(driver, 3, 0.5);

    private final JoystickButton opLeftStick = new JoystickButton(operator, XboxController.Button.kLeftStick.value);
    private final JoystickButton opRightStick = new JoystickButton(operator, XboxController.Button.kRightStick.value);
    private final JoystickButton opY = new JoystickButton(operator, XboxController.Button.kY.value);
    private final JoystickButton opA = new JoystickButton(operator, XboxController.Button.kA.value);
    private final JoystickButton opB = new JoystickButton(operator, XboxController.Button.kB.value);
    private final JoystickButton opX = new JoystickButton(operator, XboxController.Button.kX.value);
    private final POVButton povUp = new POVButton(operator, 0);
    private final POVButton povDown = new POVButton(operator, 180);
    private final POVButton povRight = new POVButton(operator, 90);
    private final POVButton povLeft = new POVButton(operator, 270);
    private final JoystickButton opStart = new JoystickButton(operator, XboxController.Button.kStart.value);
    private final JoystickButton opSelect = new JoystickButton(operator, XboxController.Button.kBack.value);
    private final JoystickButton opLeftBumper = new JoystickButton(operator, XboxController.Button.kLeftBumper.value);
    private final JoystickButton opRightBumper = new JoystickButton(operator, XboxController.Button.kRightBumper.value);
    private final AxisButton opLeftTrigger = new AxisButton(operator, 2, 0.5);
    private final AxisButton opRightTrigger = new AxisButton(operator, 3, 0.5);

    private final CannonSubsystem cannon = new CannonSubsystem();
    private final Indexer indexer = new Indexer();
    private final Intake intake = new Intake();

    



  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
        NamedCommands.registerCommand("Run cannon", new InstantCommand(() -> cannon.launchWithVolts()));
        NamedCommands.registerCommand("Index", new InstantCommand(() -> indexer.set(1.0)));
        NamedCommands.registerCommand("Stop Index", new InstantCommand(() -> indexer.set(0.0)));
        NamedCommands.registerCommand("Run Intake", new InstantCommand(() -> intake.set(1.0)));
        NamedCommands.registerCommand("PivotHome", Commands.runOnce(() -> cannon.setSetpointDegrees(22), cannon));
        NamedCommands.registerCommand("Subwoofer", Commands.runOnce(() -> cannon.setSetpointDegrees(57), cannon));
        NamedCommands.registerCommand("End cannon", new InstantCommand(() -> cannon.stopLaunchWithVolts()));
        NamedCommands.registerCommand("End Intake", new InstantCommand(() -> intake.set(0.0)));
        NamedCommands.registerCommand("Pivot Up", new InstantCommand(() -> cannon.pivotUp()));
        NamedCommands.registerCommand("Pivot Down", new InstantCommand(() -> cannon.pivotDown()));

        configureButtonBindings();

        LEDS.setColor(LEDIdentifier.DARK_BLUE);
        
  }


  private void configureButtonBindings() {
        /* Driver Buttons */
        
        //INTAKE
        driveLeftTrigger.onTrue(new InstantCommand(() -> intake.set(1.0), intake));
        driveLeftTrigger.onFalse(new InstantCommand(() -> intake.set(0.0), intake));
        driveLeftTrigger.onTrue(new InstantCommand(() -> indexer.set(1.0), indexer));
        driveLeftTrigger.onFalse(new InstantCommand(() -> indexer.set(0.0), indexer));
        opLeftBumper.whileTrue(new DriverIntakeFeedback(intake, driver, operator));

                
        driveRightTrigger.onTrue(new InstantCommand(() -> intake.set(-1.0), intake));
        driveRightTrigger.onFalse(new InstantCommand(() -> intake.set(0.0), intake));
        driveRightTrigger.onTrue(new InstantCommand(() -> indexer.set(-1.0), indexer));
        driveRightTrigger.onFalse(new InstantCommand(() -> indexer.set(0.0), indexer));
 
        //CANNON
        opX.onTrue(new InstantCommand(() -> cannon.launchWithVolts()));
        opX.onFalse(new InstantCommand(() -> cannon.stopLaunchWithVolts()));
        opX.whileFalse(Commands.runOnce(() ->cannon.setSetpointDegrees(22),cannon));
        opLeftTrigger.onTrue(new InstantCommand(() -> cannon.launchWithVolts()));
        opLeftTrigger.onFalse(new InstantCommand(() -> cannon.stopLaunchWithVolts()));

        opRightTrigger.onTrue(new InstantCommand(() -> cannon.slowLaunchWithVolts()));
        opRightTrigger.onFalse(new InstantCommand(() -> cannon.stopLaunchWithVolts()));

        povUp.onTrue(new InstantCommand(() ->cannon.pivotUp(),cannon));
        povUp.onFalse(new InstantCommand(() ->cannon.pivotStop(),cannon));
        povDown.onTrue(new InstantCommand(() ->cannon.pivotDown(),cannon));
        povDown.onFalse(new InstantCommand(() ->cannon.pivotStop(),cannon));


        
    }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    
    return null;
  }
}
