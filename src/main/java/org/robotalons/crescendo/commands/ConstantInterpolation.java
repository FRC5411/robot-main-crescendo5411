package org.robotalons.crescendo.commands;

import java.util.function.BooleanSupplier;

import org.robotalons.crescendo.subsystems.cannon.CannonSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class ConstantInterpolation extends Command{
    
    private CannonSubsystem cannon;
    private BooleanSupplier pivotToSubwoofer;
    private BooleanSupplier pivotUpOverride;
    private BooleanSupplier pivotDownOverride;
    private BooleanSupplier togglerOverride;
    private BooleanSupplier pivotToAmp;

    public ConstantInterpolation(CannonSubsystem cannon, BooleanSupplier pivotToSubwoofer, BooleanSupplier pivotToAmp, BooleanSupplier pivotUpOverride, BooleanSupplier  pivotDownOverride, BooleanSupplier toggleOverride) {

        this.cannon = cannon;
        this.pivotToSubwoofer = pivotToSubwoofer;
        this.pivotToAmp = pivotToAmp;
        this.pivotUpOverride = pivotUpOverride;
        this.pivotDownOverride = pivotDownOverride;
        this.togglerOverride = toggleOverride;
        addRequirements(cannon);
    }

    @Override
    public void execute() {
        if (pivotToSubwoofer.getAsBoolean()) { //TODO: Add in vision interpolation
        cannon.setSetpointDegrees(51);
    } else if (pivotToAmp.getAsBoolean()) {
        cannon.setSetpointDegrees(56);
    } else if (pivotUpOverride.getAsBoolean() && togglerOverride.getAsBoolean()) {
        cannon.pivotUp();
    } else if (pivotDownOverride.getAsBoolean() && togglerOverride.getAsBoolean()) {
        cannon.pivotDown();
    } else if (togglerOverride.getAsBoolean()) {
        cannon.holdPosition();
    } else {
        cannon.setSetpointDegrees(22);
    }
}

@Override
public void end(boolean interrupted) {
    cannon.setSetpointDegrees(22);
}



}
