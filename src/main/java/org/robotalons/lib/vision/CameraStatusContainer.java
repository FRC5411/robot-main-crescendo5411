// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.lib.vision;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//

import edu.wpi.first.math.geometry.Pose2d;


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
  public Pose2d RobotPose = new Pose2d();
  public Pose2d[] Deltas = new Pose2d[]{};
  public double[] Timestamps = new double[]{};
}