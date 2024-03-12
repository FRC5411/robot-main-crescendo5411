package org.robotalons.crescendo.commands;

import org.robotalons.crescendo.subsystems.intake.Intake;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RuntimeType;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;


public class DriverIntakeFeedback extends Command{
    
    private boolean bannerSensor;
    private Joystick driver;
    private Joystick operator;
    private Intake intake;

    public DriverIntakeFeedback (Intake intake, Joystick driver, Joystick operator){

        this.intake = intake;
        this.driver = driver;
        this.operator = operator;
    }

    @Override
    public void execute() {
        if (intake.getBannerSensor()) {
            driver.setRumble(RumbleType.kBothRumble, 1);
            operator.setRumble(RumbleType.kBothRumble, 1);
        } else {
            driver.setRumble(RumbleType.kBothRumble, 0);
            operator.setRumble(RumbleType.kBothRumble, 0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        driver.setRumble(RumbleType.kBothRumble, 0);
        operator.setRumble(RumbleType.kBothRumble, 0);
    }
}
