// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.crescendo.subsystems.drive;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

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
  public static final class Simulation {
    public static final Double TRANSLATIONAL_FLYWHEEL_KG_PER_METER_SQUARED = (505.597120488d); //TODO: Update
    public static final Double ROTATIONAL_FLYWHEEL_KG_PER_METER_SQUARED = (249.71004822d); //TODO: Update
    public static final Matrix<N1,N1> TRANSLATIONAL_MEASUREMENT_STDEVS = MatBuilder.fill(Nat.N1(), Nat.N1(), (0.25));
    public static final Matrix<N1,N1> ROTATIONAL_MEASUREMENT_STDEVS = MatBuilder.fill(Nat.N1(), Nat.N1(), (0.25));
    public static final FlywheelSim TRANSLATIONAL_FLYWHEEL = new FlywheelSim(
      DCMotor.getNEO((1)), 
      1 / Measurements.ROBOT_LINEAR_GEAR_RATIO, 
      TRANSLATIONAL_FLYWHEEL_KG_PER_METER_SQUARED, TRANSLATIONAL_MEASUREMENT_STDEVS);
    public static final FlywheelSim ROTATIONAL_FLYWHEEL = new FlywheelSim(
      DCMotor.getNEO((1)), 
      1 / Measurements.ROBOT_ROTATION_GEAR_RATIO, 
      ROTATIONAL_FLYWHEEL_KG_PER_METER_SQUARED, ROTATIONAL_MEASUREMENT_STDEVS);
  }

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

    public static final double DRIVE_ENCODER_ROT2METER = ROBOT_LINEAR_GEAR_RATIO * Math.PI * ROBOT_WHEEL_DIAMETER_METERS;

    public static final double ASIMUTH_ENCODER_ROT2RAD = ROBOT_ROTATION_GEAR_RATIO * 2 * Math.PI;

    public static final double DRIVE_ENCODER_RPM2METER_SEC = DRIVE_ENCODER_ROT2METER / 60;

    public static final double ASIMUTH_ENCODER_RPM2METER_SEC = ASIMUTH_ENCODER_ROT2RAD / 60;

    public static final Boolean PHOENIX_DRIVE = (false);

    public static final Double ODOMETRY_FREQUENCY = (250d);

    public static final class Modules {
      public static final class FL {
        public static final Integer LINEAR_CONTROLLER_ID = (11);
        public static final Integer ROTATIONAL_CONTROLLER_ID = (15);
        public static final Integer ABSOLUTE_ENCODER_ID = (3);
        public static final Double ROTATIONAL_P_GAIN = (2.81d);
        public static final Double ROTATIONAL_I_GAIN = (0d);
        public static final Double ROTATIONAL_D_GAIN = (0.00001d);
        public static final Double ROTATIONAL_ENCODER_OFFSET = (0.403320d);
        public static final Boolean ROTATIONAL_INVERTED = (true);
        public static final Boolean LINEAR_INVERTED = (false);
        public static final Double LINEAR_P_GAIN = (0.2d);
        public static final Double LINEAR_I_GAIN = (0d);
        public static final Double LINEAR_D_GAIN = (0d);
        public static final Double LINEAR_KS_GAIN = (0d);
        public static final Double LINEAR_KV_GAIN = (0.021d / 12);
        public static final Double LINEAR_KA_GAIN = (0d);
        public static final Integer NUMBER = (0);

      }

      public static final class FR {
        public static final Integer LINEAR_CONTROLLER_ID = (12);
        public static final Integer ROTATIONAL_CONTROLLER_ID = (16);
        public static final Integer ABSOLUTE_ENCODER_ID = (4);
        public static final Double ROTATIONAL_P_GAIN = (0.075d);
        public static final Double ROTATIONAL_I_GAIN = (0d);
        public static final Double ROTATIONAL_D_GAIN = (0.00001d);
        public static final Double ROTATIONAL_ENCODER_OFFSET = (0.298340d);
        public static final Boolean ROTATIONAL_INVERTED = (true);
        public static final Boolean LINEAR_INVERTED = (false);
        public static final Double LINEAR_P_GAIN = (0.2d);
        public static final Double LINEAR_I_GAIN = (0d);
        public static final Double LINEAR_D_GAIN = (0d);
        public static final Double LINEAR_KS_GAIN = (0d);
        public static final Double LINEAR_KV_GAIN = (0.021d / 12);
        public static final Double LINEAR_KA_GAIN = (0d);
        public static final Integer NUMBER = (1);
      }

      public static final class RL {
        public static final Integer LINEAR_CONTROLLER_ID = (13);
        public static final Integer ROTATIONAL_CONTROLLER_ID = (17);
        public static final Integer ABSOLUTE_ENCODER_ID = (5);
        public static final Double ROTATIONAL_P_GAIN = (0.075d);
        public static final Double ROTATIONAL_I_GAIN = (0d);
        public static final Double ROTATIONAL_D_GAIN = (0.00001d);
        public static final Double ROTATIONAL_ENCODER_OFFSET = (-0.164795d);
        public static final Boolean ROTATIONAL_INVERTED = (true);
        public static final Boolean LINEAR_INVERTED = (false);
        public static final Double LINEAR_P_GAIN = (0.2d);
        public static final Double LINEAR_I_GAIN = (0d);
        public static final Double LINEAR_D_GAIN = (0d);
        public static final Double LINEAR_KS_GAIN = (0d);
        public static final Double LINEAR_KV_GAIN = (0.021d / 12);
        public static final Double LINEAR_KA_GAIN = (0d);
        public static final Integer NUMBER = (2);
      }

      public static final class RR {
        public static final Integer LINEAR_CONTROLLER_ID = (14);
        public static final Integer ROTATIONAL_CONTROLLER_ID = (18);
        public static final Integer ABSOLUTE_ENCODER_ID = (6);
        public static final Double ROTATIONAL_P_GAIN = (0.075d);
        public static final Double ROTATIONAL_I_GAIN = (0d);
        public static final Double ROTATIONAL_D_GAIN = (0.00001d);
        public static final Double ROTATIONAL_ENCODER_OFFSET = (-0.412842d);
        public static final Boolean ROTATIONAL_INVERTED = (true);
        public static final Boolean LINEAR_INVERTED = (false);
        public static final Double LINEAR_P_GAIN = (0.2d);
        public static final Double LINEAR_I_GAIN = (0d);
        public static final Double LINEAR_D_GAIN = (0d);
        public static final Double LINEAR_KS_GAIN = (0d);
        public static final Double LINEAR_KV_GAIN = (0.021d /12);
        public static final Double LINEAR_KA_GAIN = (0d);
        public static final Integer NUMBER = (3);
      }
    }
  }

  public static final class Ports {
    public static final Integer GYROSCOPE_ID = (10);      
  }

  public static final class Objects {
    public static final Lock ODOMETRY_LOCK = new ReentrantLock();
  }

  public static final class Devices {}
}