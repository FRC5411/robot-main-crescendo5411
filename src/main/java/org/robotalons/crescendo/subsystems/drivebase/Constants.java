// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.crescendo.subsystems.drivebase;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.robotalons.crescendo.subsystems.drivebase.REVControllerModule.ModuleConstants;
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
    public static final Double ROBOT_WHEEL_DIAMETER_METERS = Units.inchesToMeters((4));
    public static final Double ROBOT_WHEEL_PERIMETER_METERS = ROBOT_WHEEL_DIAMETER_METERS * Math.PI;
    public static final Double ROBOT_LENGTH_METERS = Units.inchesToMeters((29));        
    public static final Double ROBOT_WIDTH_METERS = Units.inchesToMeters((29));
    public static final Double ROBOT_RADIUS_METERS = Math.hypot(ROBOT_LENGTH_METERS / (2.0), ROBOT_WIDTH_METERS / (2.0));      

    public static final Double ROBOT_MAXIMUM_LINEAR_VELOCITY = Units.feetToMeters((15.4d));
    public static final Double ROBOT_MAXIMUM_ANGULAR_VELOCITY = ROBOT_MAXIMUM_LINEAR_VELOCITY / ROBOT_RADIUS_METERS;

    public static final Double ROBOT_LINEAR_GEAR_RATIO = ((6.75));
    public static final Double ROBOT_ROTATION_GEAR_RATIO = (((150.0) / (7.0)));

    public static final Double ROBOT_TRANSLATION_KP = (1d);
    public static final Double ROBOT_TRANSLATION_KI = (1d);
    public static final Double ROBOT_TRANSLATION_KD = (1d);    

    public static final Double ROBOT_ROTATIONAL_KP = (1d);
    public static final Double ROBOT_ROTATIONAL_KI = (1d);
    public static final Double ROBOT_ROTATIONAL_KD = (1d);

    public static final Boolean PHOENIX_DRIVE = (false);

    public static final Double ODOMETRY_FREQUENCY = (250d);

    public static final class Modules {
      public static final class FL {
        public static final Integer LINEAR_CONTROLLER_ID = (11);
        public static final Integer ROTATIONAL_CONTROLLER_ID = (21);
        public static final Integer ABSOLUTE_ENCODER_ID = (31);
        public static final Double ROTATIONAL_P_GAIN = (2.8d);
        public static final Double ROTATIONAL_I_GAIN = (0d);
        public static final Double ROTATIONAL_D_GAIN = (0.00001d);
        public static final Rotation2d ROTATIONAL_ENCODER_OFFSET = Rotation2d.fromRotations((0.607422d));
        public static final Boolean ROTATIONAL_INVERTED = (false);
        public static final Boolean LINEAR_INVERTED = (false);
        public static final Double LINEAR_P_GAIN = (0.02d);
        public static final Double LINEAR_I_GAIN = (0d);
        public static final Double LINEAR_D_GAIN = (0d);
        public static final Double LINEAR_KS_GAIN = (0d);
        public static final Double LINEAR_KV_GAIN = (0.021d);
        public static final Double LINEAR_KA_GAIN = (0d);
        public static final Integer NUMBER = (0);
        public static final ModuleConstants CONSTANTS = new ModuleConstants();
        static {
          CONSTANTS.LINEAR_CONTROLLER = new CANSparkMax(LINEAR_CONTROLLER_ID, MotorType.kBrushless);
          CONSTANTS.ROTATIONAL_CONTROLLER = new CANSparkMax(ROTATIONAL_CONTROLLER_ID, MotorType.kBrushless);
          CONSTANTS.ABSOLUTE_ENCODER = new CANcoder(ABSOLUTE_ENCODER_ID);
          CONSTANTS.LINEAR_CONTROLLER_PID = new PIDController(LINEAR_P_GAIN, LINEAR_I_GAIN, LINEAR_D_GAIN);
          CONSTANTS.ROTATIONAL_CONTROLLER_PID = new PIDController(ROTATIONAL_P_GAIN, ROTATIONAL_I_GAIN, ROTATIONAL_D_GAIN);
          CONSTANTS.LINEAR_CONTROLLER_FEEDFORWARD = new SimpleMotorFeedforward(LINEAR_KS_GAIN, LINEAR_KV_GAIN, LINEAR_KA_GAIN);
          CONSTANTS.LINEAR_GEAR_RATIO = Measurements.ROBOT_LINEAR_GEAR_RATIO;
          CONSTANTS.ROTATION_GEAR_RATIO = Measurements.ROBOT_ROTATION_GEAR_RATIO;
          CONSTANTS.WHEEL_RADIUS_METERS = Measurements.ROBOT_WHEEL_DIAMETER_METERS / 2;
          CONSTANTS.ROTATIONAL_ENCODER_OFFSET = ROTATIONAL_ENCODER_OFFSET;
          CONSTANTS.LINEAR_INVERTED = LINEAR_INVERTED;
          CONSTANTS.ROTATIONAL_INVERTED = ROTATIONAL_INVERTED;
          CONSTANTS.NUMBER = NUMBER;
        }
      }

      public static final class FR {
        public static final Integer LINEAR_CONTROLLER_ID = (12);
        public static final Integer ROTATIONAL_CONTROLLER_ID = (22);
        public static final Integer ABSOLUTE_ENCODER_ID = (32);
        public static final Double ROTATIONAL_P_GAIN = (2.8d);
        public static final Double ROTATIONAL_I_GAIN = (0d);
        public static final Double ROTATIONAL_D_GAIN = (0.00001d);
        public static final Rotation2d ROTATIONAL_ENCODER_OFFSET = Rotation2d.fromRotations((0.821045d));
        public static final Boolean ROTATIONAL_INVERTED = (false);
        public static final Boolean LINEAR_INVERTED = (false);
        public static final Double LINEAR_P_GAIN = (0.02d);
        public static final Double LINEAR_I_GAIN = (0d);
        public static final Double LINEAR_D_GAIN = (0d);
        public static final Double LINEAR_KS_GAIN = (0d);
        public static final Double LINEAR_KV_GAIN = (0.021d);
        public static final Double LINEAR_KA_GAIN = (0d);
        public static final Integer NUMBER = (1);
        public static final ModuleConstants CONSTANTS = new ModuleConstants();
        static {
          CONSTANTS.LINEAR_CONTROLLER = new CANSparkMax(LINEAR_CONTROLLER_ID, MotorType.kBrushless);
          CONSTANTS.ROTATIONAL_CONTROLLER = new CANSparkMax(ROTATIONAL_CONTROLLER_ID, MotorType.kBrushless);
          CONSTANTS.ABSOLUTE_ENCODER = new CANcoder(ABSOLUTE_ENCODER_ID);
          CONSTANTS.LINEAR_CONTROLLER_PID = new PIDController(LINEAR_P_GAIN, LINEAR_I_GAIN, LINEAR_D_GAIN);
          CONSTANTS.ROTATIONAL_CONTROLLER_PID = new PIDController(ROTATIONAL_P_GAIN, ROTATIONAL_I_GAIN, ROTATIONAL_D_GAIN);
          CONSTANTS.LINEAR_CONTROLLER_FEEDFORWARD = new SimpleMotorFeedforward(LINEAR_KS_GAIN, LINEAR_KV_GAIN, LINEAR_KA_GAIN);
          CONSTANTS.LINEAR_GEAR_RATIO = Measurements.ROBOT_LINEAR_GEAR_RATIO;
          CONSTANTS.ROTATION_GEAR_RATIO = Measurements.ROBOT_ROTATION_GEAR_RATIO;
          CONSTANTS.WHEEL_RADIUS_METERS = Measurements.ROBOT_WHEEL_DIAMETER_METERS / 2;
          CONSTANTS.ROTATIONAL_ENCODER_OFFSET = ROTATIONAL_ENCODER_OFFSET;
          CONSTANTS.LINEAR_INVERTED = LINEAR_INVERTED;
          CONSTANTS.ROTATIONAL_INVERTED = ROTATIONAL_INVERTED;
          CONSTANTS.NUMBER = NUMBER;
        }
      }

      public static final class RL {
        public static final Integer LINEAR_CONTROLLER_ID = (13);
        public static final Integer ROTATIONAL_CONTROLLER_ID = (23);
        public static final Integer ABSOLUTE_ENCODER_ID = (33);
        public static final Double ROTATIONAL_P_GAIN = (2.8d);
        public static final Double ROTATIONAL_I_GAIN = (0d);
        public static final Double ROTATIONAL_D_GAIN = (0.00001d);
        public static final Rotation2d ROTATIONAL_ENCODER_OFFSET = Rotation2d.fromRotations((0.353027d));
        public static final Boolean ROTATIONAL_INVERTED = (false);
        public static final Boolean LINEAR_INVERTED = (true);
        public static final Double LINEAR_P_GAIN = (0.02d);
        public static final Double LINEAR_I_GAIN = (0d);
        public static final Double LINEAR_D_GAIN = (0d);
        public static final Double LINEAR_KS_GAIN = (0d);
        public static final Double LINEAR_KV_GAIN = (0.021d);
        public static final Double LINEAR_KA_GAIN = (0d);
        public static final Integer NUMBER = (2);
        public static final ModuleConstants CONSTANTS = new ModuleConstants();
        static {
          CONSTANTS.LINEAR_CONTROLLER = new CANSparkMax(LINEAR_CONTROLLER_ID, MotorType.kBrushless);
          CONSTANTS.ROTATIONAL_CONTROLLER = new CANSparkMax(ROTATIONAL_CONTROLLER_ID, MotorType.kBrushless);
          CONSTANTS.ABSOLUTE_ENCODER = new CANcoder(ABSOLUTE_ENCODER_ID);
          CONSTANTS.LINEAR_CONTROLLER_PID = new PIDController(LINEAR_P_GAIN, LINEAR_I_GAIN, LINEAR_D_GAIN);
          CONSTANTS.ROTATIONAL_CONTROLLER_PID = new PIDController(ROTATIONAL_P_GAIN, ROTATIONAL_I_GAIN, ROTATIONAL_D_GAIN);
          CONSTANTS.LINEAR_CONTROLLER_FEEDFORWARD = new SimpleMotorFeedforward(LINEAR_KS_GAIN, LINEAR_KV_GAIN, LINEAR_KA_GAIN);
          CONSTANTS.LINEAR_GEAR_RATIO = Measurements.ROBOT_LINEAR_GEAR_RATIO;
          CONSTANTS.ROTATION_GEAR_RATIO = Measurements.ROBOT_ROTATION_GEAR_RATIO;
          CONSTANTS.WHEEL_RADIUS_METERS = Measurements.ROBOT_WHEEL_DIAMETER_METERS / 2;
          CONSTANTS.ROTATIONAL_ENCODER_OFFSET = ROTATIONAL_ENCODER_OFFSET;
          CONSTANTS.LINEAR_INVERTED = LINEAR_INVERTED;
          CONSTANTS.ROTATIONAL_INVERTED = ROTATIONAL_INVERTED;
          CONSTANTS.NUMBER = NUMBER;
        }
      }

      public static final class RR {
        public static final Integer LINEAR_CONTROLLER_ID = (14);
        public static final Integer ROTATIONAL_CONTROLLER_ID = (24);
        public static final Integer ABSOLUTE_ENCODER_ID = (34);
        public static final Double ROTATIONAL_P_GAIN = (2.8d);
        public static final Double ROTATIONAL_I_GAIN = (0d);
        public static final Double ROTATIONAL_D_GAIN = (0.00001d);
        public static final Rotation2d ROTATIONAL_ENCODER_OFFSET = Rotation2d.fromRotations((0.483643d));
        public static final Boolean ROTATIONAL_INVERTED = (false);
        public static final Boolean LINEAR_INVERTED = (true);
        public static final Double LINEAR_P_GAIN = (0.02d);
        public static final Double LINEAR_I_GAIN = (0d);
        public static final Double LINEAR_D_GAIN = (0d);
        public static final Double LINEAR_KS_GAIN = (0d);
        public static final Double LINEAR_KV_GAIN = (0.021d);
        public static final Double LINEAR_KA_GAIN = (0d);
        public static final Integer NUMBER = (3);
        public static final ModuleConstants CONSTANTS = new ModuleConstants();
        static {
          CONSTANTS.LINEAR_CONTROLLER = new CANSparkMax(LINEAR_CONTROLLER_ID, MotorType.kBrushless);
          CONSTANTS.ROTATIONAL_CONTROLLER = new CANSparkMax(ROTATIONAL_CONTROLLER_ID, MotorType.kBrushless);
          CONSTANTS.ABSOLUTE_ENCODER = new CANcoder(ABSOLUTE_ENCODER_ID);
          CONSTANTS.LINEAR_CONTROLLER_PID = new PIDController(LINEAR_P_GAIN, LINEAR_I_GAIN, LINEAR_D_GAIN);
          CONSTANTS.ROTATIONAL_CONTROLLER_PID = new PIDController(ROTATIONAL_P_GAIN, ROTATIONAL_I_GAIN, ROTATIONAL_D_GAIN);
          CONSTANTS.LINEAR_CONTROLLER_FEEDFORWARD = new SimpleMotorFeedforward(LINEAR_KS_GAIN, LINEAR_KV_GAIN, LINEAR_KA_GAIN);
          CONSTANTS.LINEAR_GEAR_RATIO = Measurements.ROBOT_LINEAR_GEAR_RATIO;
          CONSTANTS.ROTATION_GEAR_RATIO = Measurements.ROBOT_ROTATION_GEAR_RATIO;
          CONSTANTS.WHEEL_RADIUS_METERS = Measurements.ROBOT_WHEEL_DIAMETER_METERS / 2;
          CONSTANTS.ROTATIONAL_ENCODER_OFFSET = ROTATIONAL_ENCODER_OFFSET;
          CONSTANTS.LINEAR_INVERTED = LINEAR_INVERTED;
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
      new REVControllerModule(Measurements.Modules.FL.CONSTANTS);     
    public static final Module FRONT_RIGHT_MODULE = 
      new REVControllerModule(Measurements.Modules.FR.CONSTANTS);             
    public static final Module REAR_LEFT_MODULE = 
      new REVControllerModule(Measurements.Modules.RL.CONSTANTS);        
    public static final Module REAR_RIGHT_MODULE = 
      new REVControllerModule(Measurements.Modules.RR.CONSTANTS);    
    }
  }
