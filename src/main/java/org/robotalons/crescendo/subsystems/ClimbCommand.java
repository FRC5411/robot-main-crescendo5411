// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.robotalons.crescendo.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

public class ClimbCommand extends Command {
  private PIDController controller;

  private double setpoint = 0;
  private ClimbArmSubsystem climbSS;

  public ClimbCommand(double setpoint, ClimbArmSubsystem climbSS) {
    this.setpoint = setpoint;
    this.climbSS = climbSS;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    controller = new PIDController(Constants.proportionalGain, Constants.integralGain, 0);
    controller.setTolerance(2);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double pidCalc = controller.calculate(climbSS.getAngle(), setpoint);
    climbSS.setArm(pidCalc);
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
