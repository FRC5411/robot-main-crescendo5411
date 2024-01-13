// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.lib.motion.kinematics;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//

import edu.wpi.first.math.kinematics.SwerveModuleState;

// ----------------------------------------------------[Swerve Module Second Order States]--------------------------------------------------//
/**
 * 
 * @author FRC TEAM 4481
 * 
 */
@Deprecated(forRemoval = true)
public class SwerveModuleSecondOrderStates {
  // ---------------------------------------------------------------[Fields]----------------------------------------------------------------//
  private SwerveModuleState[] ModuleStates;
  private double[] ModuleAzimuthSpeeds;
  // ------------------------------------------------------------[Constructors]-------------------------------------------------------------//
  /**
   * Swerve Module Second Order States Constructor.
   *
   * @param ModuleStates Array of Module States
   * @param ModuleAzimuthSpeeds Array of Speeds with Azimuths turn at
   */
  public SwerveModuleSecondOrderStates( SwerveModuleState[] ModuleStates, double[] ModuleAzimuthSpeeds){
       this.ModuleStates = ModuleStates;
       this.ModuleAzimuthSpeeds = ModuleAzimuthSpeeds;
    }
  // --------------------------------------------------------------[Mutators]---------------------------------------------------------------//
  public void setSwerveModuleStates(SwerveModuleState[] swerveModuleStates){
      this.ModuleStates = swerveModuleStates;
  }

  public void setModuleTurnSpeeds(double[] moduleTurnSpeeds){
      this.ModuleAzimuthSpeeds = moduleTurnSpeeds;
  }

  // --------------------------------------------------------------[Accessors]--------------------------------------------------------------//
  public SwerveModuleState[] getSwerveModuleStates(){
      return ModuleStates;
  }
  public double[] getModuleTurnSpeeds(){
      return ModuleAzimuthSpeeds;
  }
}