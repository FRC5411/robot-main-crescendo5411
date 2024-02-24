// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.robotalons.lib.climb;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import java.io.Closeable;

// -----------------------------------------------------------------[Climb]----------------------------------------------------------------//
/**
 *
 *
 * <p>Singular unit which assists in the climb of a given robot.
 * 
 */
public abstract class Climb implements Closeable {
    // --------------------------------------------------------------[Constants]--------------------------------------------------------------//
    protected final ClimbStatusContainer CLIMBSTATUS = new ClimbStatusContainer();
    // -------------------------------------------------------------[Constructors]------------------------------------------------------------//
    
    /**
     * Climb Constructor
     */
    protected Climb(){}

    // ---------------------------------------------------------------[Abstract]--------------------------------------------------------------//
    /**
     * This method is called periodically by the {@link CommandScheduler}. Useful for updating
     * subsystem-specific stte that you don't want to offload to a {@link Command}. Teams should try
     * to be consistent within their own codebases about which responsibilities will be handled by
     * Commands, and which will be handled here.
     */
    public abstract void periodic();

    /**
     * Updates the underlying signals within this module
     */
    public abstract void update();


    // --------------------------------------------------------------[Mutators]---------------------------------------------------------------//

   /**
     * Mutate PID setpoint for climb motor
     * @param SETPOINT refers to setpoint for the PID to move to
     */
    public abstract void pidSet(Double SETPOINT);

   /**
     * Set specified voltage to the climb motor
     * @param DEMAND refers to amount of voltage that should me applied to motors
     */    
    public abstract void set(Double DEMAND);

  /**
    * Calculates voltage needed for feedforward
   */    
    public abstract void calculateAppliedVoltage();


    // --------------------------------------------------------------[Accessors]--------------------------------------------------------------//
    
    /**
     *  Get temperature of climb motor
     */    
    public Double getTemperature(){
      return CLIMBSTATUS.temperature;
    }

    /**
     * Get velocity of climb motor
     */  
    public Double getVelocity(){
      return CLIMBSTATUS.velocity;
    }

    /**
     * Get posistion of climb motor
     */  
    public Double getPosistion(){
      return CLIMBSTATUS.posistion;
    }

}
