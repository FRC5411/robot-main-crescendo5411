// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.lib.vision;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//

import edu.wpi.first.math.geometry.Pose3d;

import org.littletonrobotics.junction.AutoLog;

// ---------------------------------------------------------[CameraStatusContainer]----------------------------------------------------------//
/**
 *
 *
 * <p>Loggable input reference to a specific singular unit which assists in the odometry and kinematics of a given robot.
 */
@AutoLog
public class CameraStatusContainer {
  public String Name = ("");
  public boolean Connected = (false);
  public double Latency = (0d);
  //TODO: Return robot pose, un edit this and change the abstract method once drivebase pose estimation implemented
  // public Pose3d RobotPose = new Pose3d(0, 0, 0, null);
  public Pose3d[] deltas = new Pose3d[]{};
  public double[] timestamps = new double[]{};
}