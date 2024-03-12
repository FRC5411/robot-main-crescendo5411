package org.robotalons.crescendo.commands;

import org.robotalons.crescendo.subsystems.cannon.CannonSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class InterpolateToSpeaker extends Command{
    private CannonSubsystem cannon;

    public InterpolateToSpeaker(CannonSubsystem cannon) {

        this.cannon = cannon;
        addRequirements(cannon);
    }

    @Override
    public void execute() {
        //TODO: Uncomment when interpolation added
        //cannon.interpolateSetpoint();
    }
}
