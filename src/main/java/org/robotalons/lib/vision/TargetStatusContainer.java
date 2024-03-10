// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.lib.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;

import org.littletonrobotics.junction.AutoLog;

// ---------------------------------------------------------[TargetStatusContainer]----------------------------------------------------------//
/**
 *
 *
 * <p>Loggable input reference to a specific singular unit which assists in position of targets. 
 */
@AutoLog
public class TargetStatusContainer {
  public Pose2d BestTargetPose = new Pose2d(0, 0, null);
  public Transform2d OptimalTransform = new Transform2d(0, 0, null);
  public Transform2d[] Targets = new Transform2d[]{};
  public boolean HasTargets = (false);
  public int TotalTargets = 0;
}