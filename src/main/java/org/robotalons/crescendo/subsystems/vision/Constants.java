// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.crescendo.subsystems.vision;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

// ---------------------------------------------------------------[Constants]---------------------------------------------------------------//
/**
 *
 *
 * <h1>Constants</h1>
 *
 * <p>Contains all robot-wide constants, does not contain subsystem specific constants.
 *
 * @see DrivebaseSubsystem
 */
public class Constants {
  // ------------------------------------------------------------[Internal]-------------------------------------------------------------//
  
  // Camera_1, Camera_2, and Camera_3 are all April Tag Pipeline Cameras //
  // Camera_4 is an Object Detection Camera //

  public static final class Vision {
    public static final String[] CAMERA_NAMES = new String[]{ "Camera_1", "Camera_2", "Camera_3", "Camera_4(OD)" };

    // Camera poses relative from robot base, double[] goes as [x , y, x, roll, pitch, yaw]
    public static final double[] CAMERA_1_POSE = new double[]{0, 0, 0, 0, 0, 0};
    public static final double[] CAMERA_2_POSE = new double[]{0, 0, 0, 0, 0, 0};
    public static final double[] CAMERA_3_POSE = new double[]{0, 0, 0, 0, 0, 0};
    public static final double[] CAMERA_4_POSE = new double[]{0, 0, 0, 0, 0, 0};

  }

  public static final class AprilTag {

    private static final double K_IN_2_M = 0.0254;

    // Converted to Meters for compatibility with pose classes //
    private static final double convert(double num){
      return num * K_IN_2_M;
    }


    public static final Pose3d APRIL_TAG_1_POSE = new Pose3d(convert(593.68), convert(9.68), convert(53.38), new Rotation3d(0, 0, 120));
    public static final Pose3d APRIL_TAG_2_POSE = new Pose3d(convert(637.12), convert(34.79), convert(53.38), new Rotation3d(0, 0, 120));
    public static final Pose3d APRIL_TAG_3_POSE = new Pose3d(convert(652.73), convert(196.17), convert(57.13), new Rotation3d(0, 0, 180));
    public static final Pose3d APRIL_TAG_4_POSE = new Pose3d(convert(652.73), convert(218.42), convert(57.13), new Rotation3d(0, 0, 180));
    public static final Pose3d APRIL_TAG_5_POSE = new Pose3d(convert(578.77), convert(323), convert(53.38), new Rotation3d(0, 0, 270));
    public static final Pose3d APRIL_TAG_6_POSE = new Pose3d(convert(72.5), convert(323), convert(53.38), new Rotation3d(0, 0, 270));
    public static final Pose3d APRIL_TAG_7_POSE = new Pose3d(convert(-1.5), convert(218.42), convert(57.13), new Rotation3d(0, 0, 0));
    public static final Pose3d APRIL_TAG_8_POSE = new Pose3d(convert(-1.5), convert(196.17), convert(57.13), new Rotation3d(0, 0, 0));
    public static final Pose3d APRIL_TAG_9_POSE = new Pose3d(convert(14.02), convert(34.79), convert(53.38), new Rotation3d(0, 0, 60));
    public static final Pose3d APRIL_TAG_10_POSE = new Pose3d(convert(57.54), convert(9.68), convert(53.38), new Rotation3d(0, 0, 60));
    public static final Pose3d APRIL_TAG_11_POSE = new Pose3d(convert(468.69), convert(146.19), convert(52), new Rotation3d(0, 0, 300));
    public static final Pose3d APRIL_TAG_12_POSE = new Pose3d(convert(468.69), convert(177.1), convert(52), new Rotation3d(0, 0, 60));
    public static final Pose3d APRIL_TAG_13_POSE = new Pose3d(convert(441.74), convert(161.62), convert(52), new Rotation3d(0, 0, 180));
    public static final Pose3d APRIL_TAG_14_POSE = new Pose3d(convert(209.48), convert(161.62), convert(52), new Rotation3d(0, 0, 0));
    public static final Pose3d APRIL_TAG_15_POSE = new Pose3d(convert(182.73), convert(177.1), convert(52), new Rotation3d(0, 0, 120));
    public static final Pose3d APRIL_TAG_16_POSE = new Pose3d(convert(182.73), convert(146.19), convert(52), new Rotation3d(0, 0, 240));
    
    public static final Pose3d[] APRIL_TAGS = new Pose3d[]{
      APRIL_TAG_1_POSE, APRIL_TAG_2_POSE, APRIL_TAG_3_POSE, APRIL_TAG_4_POSE,
      APRIL_TAG_5_POSE, APRIL_TAG_6_POSE, APRIL_TAG_7_POSE, APRIL_TAG_8_POSE,
      APRIL_TAG_9_POSE, APRIL_TAG_10_POSE, APRIL_TAG_11_POSE, APRIL_TAG_12_POSE,
      APRIL_TAG_13_POSE, APRIL_TAG_14_POSE, APRIL_TAG_15_POSE, APRIL_TAG_16_POSE
    };
  }
}
