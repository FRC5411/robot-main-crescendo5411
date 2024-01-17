// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.crescendo.subsystems.indexer;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import edu.wpi.first.math.util.Units;

import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import org.robotalons.lib.motion.actuators.Module;

// ---------------------------------------------------------------[Constants]---------------------------------------------------------------//
/**
 *
 *
 * <h1>Constants</h1>
 *
 * <p>Contains all indexer-wide constants, does not contain robot-wide constants.
 *
 * @see IndexerSubsystem
 */
public class Constants {
  // ------------------------------------------------------------[Internal]-------------------------------------------------------------//
  public static final class Measurements {
    public static final Double INTAKE_INDEXER_MOTOR_SPEED = 1;     //placeholder values
    public static final Double SHOOTER_INDEXER_MOTOR_SPEED = 1;    //speed for each motor

    public static final Double INTAKE_INDEXER_MOTOR_DURATION = 1;  //placeholder values
    public static final Double SHOOTER_INDEXER_MOTOR_DURATION = 1; //length of time for which each motor spins
  }

  public static final class Ports {
    public static final Integer INTAKE_INDEXER_MOTOR_ID = 0;  //placeholder
    public static final Integer SHOOTER_INDEXER_MOTOR_ID = 1; //placeholder
  }

  public static final class Objects {
    public static final Lock ODOMETRY_LOCKER = new ReentrantLock();
  }

  public static final class Devices {

  }
  
}
