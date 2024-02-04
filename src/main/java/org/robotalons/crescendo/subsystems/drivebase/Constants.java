// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.crescendo.subsystems.drivebase;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

import com.pathplanner.lib.util.PIDConstants;
import com.revrobotics.CANSparkMax;

import org.robotalons.lib.motion.actuators.archetype.SparkModule;
import org.robotalons.lib.motion.actuators.Module;
import org.robotalons.lib.motion.sensors.Gyroscope;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
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
public final class Constants {
// ------------------------------------------------------------[Internal]-------------------------------------------------------------//
  public static final class Measurements {
    public static final Double ROBOT_WHEEL_DIAMETER_METERS = Units.inchesToMeters((4d));
    public static final Double ROBOT_WHEEL_PERIMETER_METERS = ROBOT_WHEEL_DIAMETER_METERS * Math.PI;

    public static final Double ROBOT_LENGTH_METERS = Units.inchesToMeters((29.5));        
    public static final Double ROBOT_WIDTH_METERS = Units.inchesToMeters((29.5));
    public static final Double ROBOT_RADIUS_METERS = Math.hypot(ROBOT_LENGTH_METERS / (2.0), ROBOT_WIDTH_METERS / (2.0)); 

    public static final Double ROBOT_MAXIMUM_LINEAR_VELOCITY = Units.feetToMeters((16.6d));
    public static final Double ROBOT_MAXIMUM_ANGULAR_VELOCITY = ROBOT_MAXIMUM_LINEAR_VELOCITY / ROBOT_RADIUS_METERS;

    public static final Double ROBOT_LINEAR_GEAR_RATIO = ((6.12d));
    public static final Double ROBOT_ROTATION_GEAR_RATIO = (((150d) / (7d)));

    public static final Boolean PHOENIX_DRIVE = (false);

    public static final Double ODOMETRY_FREQUENCY = (250d);

    public static final class Modules {
      public static final class FL {
        public static final Integer LINEAR_CONTROLLER_ID = (11);
        public static final Integer ROTATIONAL_CONTROLLER_ID = (15);
        public static final Integer ABSOLUTE_ENCODER_ID = (3);
        public static final Double ROTATIONAL_P_GAIN = (3.5d);
        public static final Double ROTATIONAL_I_GAIN = (0d);
        public static final Double ROTATIONAL_D_GAIN = (0.00001d);
        public static final Double ROTATIONAL_ENCODER_OFFSET = (0.403320d + ((1/2) * Math.PI));
        public static final Boolean ROTATIONAL_INVERTED = (true);
        public static final Boolean LINEAR_INVERTED = (false);
        public static final Double LINEAR_P_GAIN = (0.2d);
        public static final Double LINEAR_I_GAIN = (0d);
        public static final Double LINEAR_D_GAIN = (0d);
        public static final Double LINEAR_KS_GAIN = (0.0001d);
        public static final Double LINEAR_KV_GAIN = (0.21d);
        public static final Double LINEAR_KA_GAIN = (0.0001d);
        public static final Integer NUMBER = (0);
        public static final SparkModule.Constants CONSTANTS = new SparkModule.Constants();
        static {
          CONSTANTS.TRANSLATIONAL_CONTROLLER_PORT = LINEAR_CONTROLLER_ID;
          CONSTANTS.ROTATIONAL_CONTROLLER_PORT = ROTATIONAL_CONTROLLER_ID;
          CONSTANTS.ABSOLUTE_ENCODER_PORT = ABSOLUTE_ENCODER_ID;
          CONSTANTS.TRANSLATIONAL_PID_CONSTANTS = new PIDConstants(LINEAR_P_GAIN, LINEAR_I_GAIN, LINEAR_D_GAIN);
          CONSTANTS.ROTATIONAL_PID_CONSTANTS = new PIDConstants(ROTATIONAL_P_GAIN, ROTATIONAL_I_GAIN, ROTATIONAL_D_GAIN);
          CONSTANTS.TRANSLATIONAL_KS_GAIN = LINEAR_KS_GAIN;
          CONSTANTS.TRANSLATIONAL_KV_GAIN = LINEAR_KV_GAIN;
          CONSTANTS.TRANSLATIONAL_KA_GAIN = LINEAR_KA_GAIN;
          CONSTANTS.TRANSLATIONAL_GEAR_RATIO = Measurements.ROBOT_LINEAR_GEAR_RATIO;
          CONSTANTS.ROTATIONAL_GEAR_RATIO = Measurements.ROBOT_ROTATION_GEAR_RATIO;
          CONSTANTS.WHEEL_RADIUS_METERS = Measurements.ROBOT_WHEEL_DIAMETER_METERS / 2;
          CONSTANTS.STATUS_PROVIDER = org.robotalons.crescendo.Constants.Odometry.REV_ODOMETRY_THREAD;
          CONSTANTS.TRANSLATIONAL_MAXIMUM_VELOCITY_METERS = Measurements.ROBOT_MAXIMUM_LINEAR_VELOCITY;
          CONSTANTS.ROTATIONAL_MAXIMUM_VELOCITY_METERS = Measurements.ROBOT_MAXIMUM_ANGULAR_VELOCITY;
          CONSTANTS.ROTATIONAL_ENCODER_OFFSET = Rotation2d.fromRotations(ROTATIONAL_ENCODER_OFFSET);
          CONSTANTS.TRANSLATIONAL_INVERTED = LINEAR_INVERTED;
          CONSTANTS.ROTATIONAL_INVERTED = ROTATIONAL_INVERTED;
          CONSTANTS.NUMBER = NUMBER;
        }
      }

      public static final class FR {
        public static final Integer LINEAR_CONTROLLER_ID = (12);
        public static final Integer ROTATIONAL_CONTROLLER_ID = (16);
        public static final Integer ABSOLUTE_ENCODER_ID = (4);
        public static final Double ROTATIONAL_P_GAIN = (3.5d);
        public static final Double ROTATIONAL_I_GAIN = (0d);
        public static final Double ROTATIONAL_D_GAIN = (0.00001d);
        public static final Double ROTATIONAL_ENCODER_OFFSET = (0.298340d + ((1/2) * Math.PI));
        public static final Boolean ROTATIONAL_INVERTED = (true);
        public static final Boolean LINEAR_INVERTED = (false);
        public static final Double LINEAR_P_GAIN = (0.2d);
        public static final Double LINEAR_I_GAIN = (0d);
        public static final Double LINEAR_D_GAIN = (0d);
        public static final Double LINEAR_KS_GAIN = (0.0001d);
        public static final Double LINEAR_KV_GAIN = (0.21d);
        public static final Double LINEAR_KA_GAIN = (0.0001d);
        public static final Integer NUMBER = (0);
        public static final SparkModule.Constants CONSTANTS = new SparkModule.Constants();
        static {
          CONSTANTS.TRANSLATIONAL_CONTROLLER_PORT = LINEAR_CONTROLLER_ID;
          CONSTANTS.ROTATIONAL_CONTROLLER_PORT = ROTATIONAL_CONTROLLER_ID;
          CONSTANTS.ABSOLUTE_ENCODER_PORT = ABSOLUTE_ENCODER_ID;
          CONSTANTS.TRANSLATIONAL_PID_CONSTANTS = new PIDConstants(LINEAR_P_GAIN, LINEAR_I_GAIN, LINEAR_D_GAIN);
          CONSTANTS.ROTATIONAL_PID_CONSTANTS = new PIDConstants(ROTATIONAL_P_GAIN, ROTATIONAL_I_GAIN, ROTATIONAL_D_GAIN);
          CONSTANTS.TRANSLATIONAL_KS_GAIN = LINEAR_KS_GAIN;
          CONSTANTS.TRANSLATIONAL_KV_GAIN = LINEAR_KV_GAIN;
          CONSTANTS.TRANSLATIONAL_KA_GAIN = LINEAR_KA_GAIN;
          CONSTANTS.TRANSLATIONAL_GEAR_RATIO = Measurements.ROBOT_LINEAR_GEAR_RATIO;
          CONSTANTS.ROTATIONAL_GEAR_RATIO = Measurements.ROBOT_ROTATION_GEAR_RATIO;
          CONSTANTS.WHEEL_RADIUS_METERS = Measurements.ROBOT_WHEEL_DIAMETER_METERS / 2;
          CONSTANTS.STATUS_PROVIDER = org.robotalons.crescendo.Constants.Odometry.REV_ODOMETRY_THREAD;
          CONSTANTS.TRANSLATIONAL_MAXIMUM_VELOCITY_METERS = Measurements.ROBOT_MAXIMUM_LINEAR_VELOCITY;
          CONSTANTS.ROTATIONAL_MAXIMUM_VELOCITY_METERS = Measurements.ROBOT_MAXIMUM_ANGULAR_VELOCITY;
          CONSTANTS.ROTATIONAL_ENCODER_OFFSET = Rotation2d.fromRotations(ROTATIONAL_ENCODER_OFFSET);
          CONSTANTS.TRANSLATIONAL_INVERTED = LINEAR_INVERTED;
          CONSTANTS.ROTATIONAL_INVERTED = ROTATIONAL_INVERTED;
          CONSTANTS.NUMBER = NUMBER;
        }
      }

      public static final class RL {
        public static final Integer LINEAR_CONTROLLER_ID = (13);
        public static final Integer ROTATIONAL_CONTROLLER_ID = (17);
        public static final Integer ABSOLUTE_ENCODER_ID = (5);
        public static final Double ROTATIONAL_P_GAIN = (3.5d);
        public static final Double ROTATIONAL_I_GAIN = (0d);
        public static final Double ROTATIONAL_D_GAIN = (0.00001d);
        public static final Double ROTATIONAL_ENCODER_OFFSET = (-0.164795d + ((1/2) * Math.PI));
        public static final Boolean ROTATIONAL_INVERTED = (true);
        public static final Boolean LINEAR_INVERTED = (false);
        public static final Double LINEAR_P_GAIN = (0.2d);
        public static final Double LINEAR_I_GAIN = (0d);
        public static final Double LINEAR_D_GAIN = (0d);
        public static final Double LINEAR_KS_GAIN = (0.0001d);
        public static final Double LINEAR_KV_GAIN = (0.21d);
        public static final Double LINEAR_KA_GAIN = (0.0001d);
        public static final Integer NUMBER = (0);
        public static final SparkModule.Constants CONSTANTS = new SparkModule.Constants();
        static {
          CONSTANTS.TRANSLATIONAL_CONTROLLER_PORT = LINEAR_CONTROLLER_ID;
          CONSTANTS.ROTATIONAL_CONTROLLER_PORT = ROTATIONAL_CONTROLLER_ID;
          CONSTANTS.ABSOLUTE_ENCODER_PORT = ABSOLUTE_ENCODER_ID;
          CONSTANTS.TRANSLATIONAL_PID_CONSTANTS = new PIDConstants(LINEAR_P_GAIN, LINEAR_I_GAIN, LINEAR_D_GAIN);
          CONSTANTS.ROTATIONAL_PID_CONSTANTS = new PIDConstants(ROTATIONAL_P_GAIN, ROTATIONAL_I_GAIN, ROTATIONAL_D_GAIN);
          CONSTANTS.TRANSLATIONAL_KS_GAIN = LINEAR_KS_GAIN;
          CONSTANTS.TRANSLATIONAL_KV_GAIN = LINEAR_KV_GAIN;
          CONSTANTS.TRANSLATIONAL_KA_GAIN = LINEAR_KA_GAIN;
          CONSTANTS.TRANSLATIONAL_GEAR_RATIO = Measurements.ROBOT_LINEAR_GEAR_RATIO;
          CONSTANTS.ROTATIONAL_GEAR_RATIO = Measurements.ROBOT_ROTATION_GEAR_RATIO;
          CONSTANTS.WHEEL_RADIUS_METERS = Measurements.ROBOT_WHEEL_DIAMETER_METERS / 2;
          CONSTANTS.STATUS_PROVIDER = org.robotalons.crescendo.Constants.Odometry.REV_ODOMETRY_THREAD;
          CONSTANTS.TRANSLATIONAL_MAXIMUM_VELOCITY_METERS = Measurements.ROBOT_MAXIMUM_LINEAR_VELOCITY;
          CONSTANTS.ROTATIONAL_MAXIMUM_VELOCITY_METERS = Measurements.ROBOT_MAXIMUM_ANGULAR_VELOCITY;
          CONSTANTS.ROTATIONAL_ENCODER_OFFSET = Rotation2d.fromRotations(ROTATIONAL_ENCODER_OFFSET);
          CONSTANTS.TRANSLATIONAL_INVERTED = LINEAR_INVERTED;
          CONSTANTS.ROTATIONAL_INVERTED = ROTATIONAL_INVERTED;
          CONSTANTS.NUMBER = NUMBER;
        }
      }

      public static final class RR {
        public static final Integer LINEAR_CONTROLLER_ID = (14);
        public static final Integer ROTATIONAL_CONTROLLER_ID = (18);
        public static final Integer ABSOLUTE_ENCODER_ID = (6);
        public static final Double ROTATIONAL_P_GAIN = (3.5d);
        public static final Double ROTATIONAL_I_GAIN = (0d);
        public static final Double ROTATIONAL_D_GAIN = (0.00001d);
        public static final Double ROTATIONAL_ENCODER_OFFSET = (-0.412842d + ((1/2) * Math.PI));
        public static final Boolean ROTATIONAL_INVERTED = (true);
        public static final Boolean LINEAR_INVERTED = (false);
        public static final Double LINEAR_P_GAIN = (0.2d);
        public static final Double LINEAR_I_GAIN = (0d);
        public static final Double LINEAR_D_GAIN = (0d);
        public static final Double LINEAR_KS_GAIN = (0.0001d);
        public static final Double LINEAR_KV_GAIN = (0.21d);
        public static final Double LINEAR_KA_GAIN = (0.0001d);
        public static final Integer NUMBER = (0);
        public static final SparkModule.Constants CONSTANTS = new SparkModule.Constants();
        static {
          CONSTANTS.TRANSLATIONAL_CONTROLLER_PORT = LINEAR_CONTROLLER_ID;
          CONSTANTS.ROTATIONAL_CONTROLLER_PORT = ROTATIONAL_CONTROLLER_ID;
          CONSTANTS.ABSOLUTE_ENCODER_PORT = ABSOLUTE_ENCODER_ID;
          CONSTANTS.TRANSLATIONAL_PID_CONSTANTS = new PIDConstants(LINEAR_P_GAIN, LINEAR_I_GAIN, LINEAR_D_GAIN);
          CONSTANTS.ROTATIONAL_PID_CONSTANTS = new PIDConstants(ROTATIONAL_P_GAIN, ROTATIONAL_I_GAIN, ROTATIONAL_D_GAIN);
          CONSTANTS.TRANSLATIONAL_KS_GAIN = LINEAR_KS_GAIN;
          CONSTANTS.TRANSLATIONAL_KV_GAIN = LINEAR_KV_GAIN;
          CONSTANTS.TRANSLATIONAL_KA_GAIN = LINEAR_KA_GAIN;
          CONSTANTS.TRANSLATIONAL_GEAR_RATIO = Measurements.ROBOT_LINEAR_GEAR_RATIO;
          CONSTANTS.ROTATIONAL_GEAR_RATIO = Measurements.ROBOT_ROTATION_GEAR_RATIO;
          CONSTANTS.WHEEL_RADIUS_METERS = Measurements.ROBOT_WHEEL_DIAMETER_METERS / 2;
          CONSTANTS.STATUS_PROVIDER = org.robotalons.crescendo.Constants.Odometry.REV_ODOMETRY_THREAD;
          CONSTANTS.TRANSLATIONAL_MAXIMUM_VELOCITY_METERS = Measurements.ROBOT_MAXIMUM_LINEAR_VELOCITY;
          CONSTANTS.ROTATIONAL_MAXIMUM_VELOCITY_METERS = Measurements.ROBOT_MAXIMUM_ANGULAR_VELOCITY;
          CONSTANTS.ROTATIONAL_ENCODER_OFFSET = Rotation2d.fromRotations(ROTATIONAL_ENCODER_OFFSET);
          CONSTANTS.TRANSLATIONAL_INVERTED = LINEAR_INVERTED;
          CONSTANTS.ROTATIONAL_INVERTED = ROTATIONAL_INVERTED;
          CONSTANTS.NUMBER = NUMBER;
        }
      }
    }
  }

  public static final class Ports {
  public static final Integer GYROSCOPE_ID = (10);      
  }

  public static final class Objects {
    public static final Lock ODOMETRY_LOCK = new ReentrantLock();
  }

  public static final class Devices {
  public static final Gyroscope GYROSCOPE = 
    new PigeonGyroscope(Measurements.PHOENIX_DRIVE);
  public static final Module FRONT_LEFT_MODULE = 
    new SparkModule<CANSparkMax>(Measurements.Modules.FL.CONSTANTS);     
  public static final Module FRONT_RIGHT_MODULE = 
    new SparkModule<CANSparkMax>(Measurements.Modules.FR.CONSTANTS);             
  public static final Module REAR_LEFT_MODULE = 
    new SparkModule<CANSparkMax>(Measurements.Modules.RL.CONSTANTS);        
  public static final Module REAR_RIGHT_MODULE = 
    new SparkModule<CANSparkMax>(Measurements.Modules.RR.CONSTANTS);    
  }
}
