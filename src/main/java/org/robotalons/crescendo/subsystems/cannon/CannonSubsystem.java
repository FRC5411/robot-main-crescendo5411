// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.crescendo.subsystems.cannon;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import org.robotalons.crescendo.subsystems.cannon.Constants.Measurements;
import org.robotalons.crescendo.subsystems.cannon.Constants.Ports;
import org.robotalons.lib.motion.trajectory.TrajectoryManager;


// ------------------------------------------------------------[Cannon Subsystem]-----------------------------------------------------------//
/**
 *
 *
 * <h1>CannonSubsystem</h1>
 *
 * <p>Utility class which controls the firing of objects to a given target, based on the current angle in radians, distance to target, and 
 * robot drivebase states.<p>
 * 
 * @see SubsystemBase
 * @see org.robotalons.crescendo.RobotContainer RobotContainer
 */
public class CannonSubsystem extends SubsystemBase {
  // --------------------------------------------------------------[Constants]-------------------------------------------------------------- //
  public static final Pair<CANSparkMax,CANSparkMax> FIRING_CONTROLLERS;
  public static final SparkMaxPIDController FIRING_CONTROLLER_PID;
  public static final RelativeEncoder FIRING_ENCODER;

  public static final CANSparkMax PIVOT_CONTROLLER;
  public static final SparkMaxPIDController PIVOT_CONTROLLER_PID;
  public static final RelativeEncoder PIVOT_ENCODER;
  // ---------------------------------------------------------------[Fields]---------------------------------------------------------------- //
  private static CannonSubsystem Instance;
    // ------------------------------------------------------------[Constructors]----------------------------------------------------------- //
  /** 
   * Cannon Subsystem Constructor 
   */
  private CannonSubsystem() {} static {
    
    FIRING_CONTROLLERS = new Pair<CANSparkMax,CANSparkMax>(
      new CANSparkMax(Ports.FIRING_CONTROLLER_LEFT_ID, MotorType.kBrushless), 
      new CANSparkMax(Ports.FIRING_CONTROLLER_RIGHT_ID, MotorType.kBrushless));
    FIRING_CONTROLLER_PID = FIRING_CONTROLLERS.getFirst().getPIDController();
    FIRING_ENCODER = FIRING_CONTROLLERS.getFirst().getEncoder();
    FIRING_CONTROLLER_PID.setP(Measurements.FIRING_P_GAIN);
    FIRING_CONTROLLER_PID.setI(Measurements.FIRING_I_GAIN);
    FIRING_CONTROLLER_PID.setD(Measurements.FIRING_D_GAIN);
    FIRING_CONTROLLERS.getSecond().setInverted((true));
    FIRING_CONTROLLERS.getSecond().follow(FIRING_CONTROLLERS.getFirst());

    PIVOT_CONTROLLER = new CANSparkMax(Ports.PIVOT_CONTROLLER_ID, MotorType.kBrushless);
    PIVOT_CONTROLLER_PID = PIVOT_CONTROLLER.getPIDController();
    PIVOT_CONTROLLER_PID.setP(Measurements.PIVOT_I_GAIN);
    PIVOT_CONTROLLER_PID.setI(Measurements.PIVOT_I_GAIN);
    PIVOT_CONTROLLER_PID.setD(Measurements.PIVOT_D_GAIN);
    PIVOT_ENCODER = PIVOT_CONTROLLER.getEncoder();
  
    PIVOT_CONTROLLER_PID.setPositionPIDWrappingEnabled((true));
    PIVOT_CONTROLLER_PID.setPositionPIDWrappingMaxInput(Math.PI);
    PIVOT_CONTROLLER_PID.setPositionPIDWrappingMinInput(-Math.PI);
    PIVOT_CONTROLLER_PID.setOutputRange(
      -(1), 
       (1));
       PIVOT_CONTROLLER_PID.setFeedbackDevice(PIVOT_ENCODER);
    TrajectoryManager.getInstance();
  }
  
  // ---------------------------------------------------------------[Methods]--------------------------------------------------------------- //
  @Override
  public synchronized void periodic() {
    Constants.Objects.ODOMETRY_LOCKER.lock();

    Constants.Objects.ODOMETRY_LOCKER.unlock();
  }
  

  /** 
   * Closes this instance and all held resources immediately 
   */
  public synchronized void close() {
    
  }
  // --------------------------------------------------------------[Mutators]--------------------------------------------------------------- //

  // --------------------------------------------------------------[Accessors]-------------------------------------------------------------- //
  /**
   * Retrieves the existing instance of this static utility class
   * @return Utility class's instance
   */
  public static synchronized CannonSubsystem getInstance() {
      if (java.util.Objects.isNull(Instance))
          Instance = new CannonSubsystem();
      return Instance;
  }
}
