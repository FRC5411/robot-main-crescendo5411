// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.lib.vision;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//


import java.util.ArrayList;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

// ---------------------------------------------------------[TargetStatusContainer]----------------------------------------------------------//
/**
 *
 *
 * <p>Loggable input reference to a specific singular unit which assists in posistion of targets. 
 */
@AutoLog
public class TargetStatusContainer {
  public Pose3d BestTargetPose = new Pose3d(0, 0, 0, null);
  public Transform3d BestTargetTransform = new Transform3d(0, 0, 0, null);
  public ArrayList<Transform3d> Targets = new ArrayList<Transform3d>();
  public Boolean HasTargets = (false);
  public Integer NumTargets = 0;
}