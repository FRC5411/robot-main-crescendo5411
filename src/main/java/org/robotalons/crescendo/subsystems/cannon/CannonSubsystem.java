// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.crescendo.subsystems.cannon;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.robotalons.crescendo.subsystems.cannon.Constants.Devices;
import org.robotalons.lib.roller.Roller;
// ------------------------------------------------------------[Cannon Subsystem]-----------------------------------------------------------//
/**
 *
 *
 * <h1>CannonSubsystem</h1>
 *
 * <p>Utility class which controls the firing of objects to a given target, based on the current angle in radians, distance to target, and 
 * robot drivebase states.<p>
 * 
 * @see SubsystemBase
 * @see org.robotalons.crescendo.RobotContainer RobotContainer
 */
public class CannonSubsystem extends SubsystemBase {
  // --------------------------------------------------------------[Constants]-------------------------------------------------------------- //
  private static final Roller DIRECTIONAL_CONTROLLER;
  private static final Roller LAUNCH_CONTROLLER;
  private static OrientationMode Control_Mode;
  // ---------------------------------------------------------------[Fields]---------------------------------------------------------------- //
  private static CannonSubsystem Instance;
  
  // ------------------------------------------------------------[Constructors]------------------------------------------------------------- //
  /** 
   * Cannon Subsystem Constructor 
   */
  private CannonSubsystem() {} static {
    DIRECTIONAL_CONTROLLER = Devices.DIRECTIONAL_CONTROLLER;
    LAUNCH_CONTROLLER = Devices.LAUNCH_CONTROLLER;
    Control_Mode = OrientationMode.ROBOT_ORIENTED;

    // DIRECTIONAL_ENCODER = DIRECTIONAL_CONTROLLER.getEncoder();
    // LAUNCH_ENCODER = LAUNCH_CONTROLLER.getEncoder();
  }
  
  // ---------------------------------------------------------------[Methods]--------------------------------------------------------------- //
  @Override
  public synchronized void periodic() {
    Constants.Objects.ODOMETRY_LOCKER.lock();
    DIRECTIONAL_CONTROLLER.periodic();
    
    LAUNCH_CONTROLLER.periodic();
    Constants.Objects.ODOMETRY_LOCKER.lock();
  }
  
  /** 
   * Closes this instance and all held resources immediately 
   */
  public synchronized void close() {
    
  }
  
  // --------------------------------------------------------------[Internal]--------------------------------------------------------------- //
  public enum OrientationMode {
    OBJECT_ORIENTED,    
    ROBOT_ORIENTED,
    FIELD_ORIENTED,
  }
  // --------------------------------------------------------------[Mutators]--------------------------------------------------------------- //
   public static synchronized void set(double DirectionalDemand, double LaunchDemand) {
    DIRECTIONAL_CONTROLLER.setVoltage(DirectionalDemand);
    LAUNCH_CONTROLLER.setVoltage(LaunchDemand);
  }
  
  // --------------------------------------------------------------[Accessors]-------------------------------------------------------------- //
  /**
   * Retrieves the existing instance of this static utility class
   * @return Utility class's instance
   */
  public static synchronized CannonSubsystem getInstance() {
      if (java.util.Objects.isNull(Instance))
          Instance = new CannonSubsystem();
      return Instance;
  }
}
