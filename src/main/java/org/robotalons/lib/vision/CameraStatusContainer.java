// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.lib.vision;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;

import org.littletonrobotics.junction.AutoLog;

// ---------------------------------------------------------[CameraStatusContainer]----------------------------------------------------------//
/**
 *
 *
 * <p>Loggable input reference to a specific singular unit which assists in the odometry and kinematics of a given robot.
 */
@AutoLog
public class CameraStatusContainer {
  public volatile boolean Connected = (false);
  public volatile double Latency = (-1);
  public volatile Pose2d Robot = new Pose2d();
  public volatile Pose2d OptimalTargetPose = new Pose2d();
  public volatile Transform2d OptimalTargetTransform = new Transform2d();
  public volatile Transform2d[] RealizedTargets = new Transform2d[]{};
  public volatile Pose2d[] Deltas = new Pose2d[]{};
  public volatile double[] Timestamps = new double[]{};
} 