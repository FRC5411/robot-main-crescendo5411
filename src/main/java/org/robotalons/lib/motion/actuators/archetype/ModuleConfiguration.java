// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.lib.motion.actuators.archetype;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import com.pathplanner.lib.util.PIDConstants;
import com.revrobotics.RelativeEncoder;

import org.robotalons.lib.motion.actuators.Module;
import org.robotalons.lib.motion.utilities.OdometryThread;

import java.util.function.DoubleSupplier;
// ----------------------------------------------------------[Module Configuration]---------------------------------------------------------//
/**
 *
 *
 * <h1>ModuleConfiguration</h1>
 *
 * <p>Configuration Container. </p>
 * 
 * @see Module
 */
public class ModuleConfiguration<Controller> extends Module.Constants {
  public OdometryThread<DoubleSupplier> STATUS_PROVIDER;
  public RelativeEncoder TRANSLATIONAL_ENCODER;
  public RelativeEncoder ROTATIONAL_ENCODER;
  public PIDConstants TRANSLATIONAL_PID_CONSTANTS;
  public PIDConstants ROTATIONAL_PID_CONSTANTS;  
  public Controller TRANSLATIONAL_CONTROLLER;
  public Controller ROTATIONAL_CONTROLLER;
  public Integer ABSOLUTE_ENCODER_PORT;      
  public String ABSOLUTE_ENCODER_BUS;
  public Double TRANSLATIONAL_KS_GAIN;
  public Double TRANSLATIONAL_KV_GAIN;
  public Double TRANSLATIONAL_KA_GAIN;  
}
