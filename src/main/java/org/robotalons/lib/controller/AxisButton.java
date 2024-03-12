package org.robotalons.lib.controller;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class AxisButton extends Trigger {

    private final GenericHID joystick;
    private final int axis;
    private final double targetVal;

    public AxisButton(GenericHID joystick, int axis, double targetVal) {
        super(() -> Math.abs(joystick.getRawAxis(axis)) > targetVal);
        this.joystick = joystick;
        this.axis = axis;
        this.targetVal = targetVal;
    }   

    public boolean get() {
        return Math.abs(joystick.getRawAxis(axis)) > targetVal;
    }
    
}