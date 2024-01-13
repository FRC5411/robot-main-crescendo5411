// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.crescendo.subsystems.drivebase;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//

import edu.wpi.first.math.util.Units;

import org.robotalons.crescendo.Constants.Subsystems;
import org.robotalons.lib.motion.actuators.CommonModule;
import org.robotalons.lib.motion.sensors.CommonGyroscope;

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
        public static final Double ROBOT_WHEEL_DIAMETER_METERS = Units.inchesToMeters((4));
        public static final Double ROBOT_WHEEL_PERIMETER_METERS = ROBOT_WHEEL_DIAMETER_METERS * Math.PI;
        public static final Double ROBOT_LENGTH_METERS = Units.inchesToMeters((29));        
        public static final Double ROBOT_WIDTH_METERS = Units.inchesToMeters((29));
        public static final Double ROBOT_RADIUS_METERS = Math.hypot(ROBOT_LENGTH_METERS / (2.0), ROBOT_WIDTH_METERS / (2.0));      

        public static final Double ROBOT_MAXIMUM_LINEAR_VELOCITY = Units.feetToMeters((15.4));
        public static final Double ROBOT_MAXIMUM_ANGULAR_VELOCITY = ROBOT_MAXIMUM_LINEAR_VELOCITY / ROBOT_RADIUS_METERS;

        public static final Double ROBOT_TRANSLATION_KP = (0d);
        public static final Double ROBOT_TRANSLATION_KI = (0d);
        public static final Double ROBOT_TRANSLATION_KD = (0d);    

        public static final Double ROBOT_ROTATIONAL_KP = (0d);
        public static final Double ROBOT_ROTATIONAL_KI = (0d);
        public static final Double ROBOT_ROTATIONAL_KD = (0d);
    }

    public static final class Ports {
        public static final Integer GYROSCOPE_ID = (0);
        public static final Integer PATHPLANNER_SERVER = (6969);        
    }

    public static final class Objects {
        public static final Lock ODOMETRY_LOCKER = new ReentrantLock();
    }

    public static final class Devices {
      public static final CommonGyroscope GYROSCOPE = 
        (Subsystems.IS_REAL_ROBOT)?
        (null):
        (null);
      public static final CommonModule FRONT_LEFT_MODULE = 
        (Subsystems.IS_REAL_ROBOT)? 
        (null): 
        (null);        
      public static final CommonModule FRONT_RIGHT_MODULE = 
        (Subsystems.IS_REAL_ROBOT)? 
        (null): 
        (null);           
      public static final CommonModule REAR_LEFT_MODULE = 
        (Subsystems.IS_REAL_ROBOT)? 
        (null): 
        (null);       
      public static final CommonModule REAR_RIGHT_MODULE = 
        (Subsystems.IS_REAL_ROBOT)? 
        (null): 
        (null);    
    }
  }
