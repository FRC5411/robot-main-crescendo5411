// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.lib.motion.actuators;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import edu.wpi.first.math.geometry.Rotation2d;

import org.littletonrobotics.junction.AutoLog;

// -------------------------------------------------------------[Common Module]-------------------------------------------------------------//
/**
 *
 *
 * <p>Loggable input reference to a specific singular unit which assists in the control of motion throughout the competition field.
 * 
 * @see CommonModule
 */
@AutoLog
public class CommonModuleData {
  public double Linear_Position_Radians = 0d;
  public double Linear_Velocity_Radians_Second = 0d;
  public double Linear_Applied_Voltage = 0d;
  public double[] Linear_Current_Amperage = new double[] {};

  public Rotation2d Azimuth_Absolute_Position = new Rotation2d();
  public Rotation2d Azimuth_Relative_Position = new Rotation2d();
  public double Azimuth_Velocity_Radians_Second = 0d;
  public double Azimuth_Applied_Voltage = 0d;
  public double[] Azimuth_Applied_Amperage = new double[] {};

  public double[] Odometry_Linear_Positions_Radians = new double[] {};
  public Rotation2d[] Odometry_Azimuth_Positions = new Rotation2d[] {};
}
