// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.crescendo.subsystems.cannon;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import org.littletonrobotics.junction.AutoLogOutput;
import org.robotalons.crescendo.subsystems.cannon.Constants.Measurements;
import org.robotalons.crescendo.subsystems.cannon.Constants.Ports;

import java.util.function.Function;

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
  public static final Function<Rotation2d,Double> OBJECT_HORIZONTAL_AREA_FUNCTION;
  public static final Function<Rotation2d,Double> OBJECT_VERTICAL_AREA_FUNCTION;

  public static final Pair<CANSparkMax,CANSparkMax> FIRING_CONTROLLERS;
  public static final SparkMaxPIDController FIRING_CONTROLLER_PID;
  public static final RelativeEncoder FIRING_ENCODER;

  public static final CANSparkMax PIVOT_CONTROLLER;
  public static final SparkMaxPIDController PIVOT_CONTROLLER_PID;
  public static final RelativeEncoder PIVOT_ENCODER;
  // ---------------------------------------------------------------[Fields]---------------------------------------------------------------- //
  private static CannonSubsystem Instance;
  private static Double CurrentTime;
    // ------------------------------------------------------------[Constructors]----------------------------------------------------------- //
  /** 
   * Cannon Subsystem Constructor 
   */
  private CannonSubsystem() {} static {
    OBJECT_HORIZONTAL_AREA_FUNCTION = (final Rotation2d Theta) -> 
    (Math.PI * (Measurements.OBJECT_OUTER_RADIUS - Measurements.OBJECT_INNER_RADIUS)) * 
     (Math.abs(Math.sin(Theta.getRadians())) + 1) * Math.PI * Measurements.OBJECT_OUTER_RADIUS;
    OBJECT_VERTICAL_AREA_FUNCTION = (final Rotation2d Theta) -> 
    (Math.PI * (Measurements.OBJECT_OUTER_RADIUS - Measurements.OBJECT_INNER_RADIUS)) * 
     (Math.abs(Math.cos(Theta.getRadians())) + 1) * Math.PI * Measurements.OBJECT_OUTER_RADIUS;
    
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
  }
  
  // ---------------------------------------------------------------[Methods]--------------------------------------------------------------- //
  @Override
  public synchronized void periodic() {
    Constants.Objects.ODOMETRY_LOCKER.lock();

    Constants.Objects.ODOMETRY_LOCKER.unlock();
  }
   
  /**
   * Calculates the discretization timestep, {@code dt}, at this current time based on the FPGA clock.
   * @return Double representation of the time passed between now and the last timestep.
   */
  @AutoLogOutput(key = "Cannon/DiscretizationTimestamp")
  private static synchronized Double discretize() {
    var DiscretizationTimestep = (0.0);
    if (CurrentTime.equals((0.0))) {
      DiscretizationTimestep = ((1.0) / (50.0));
    } else {
      var MeasuredTime = Timer.getFPGATimestamp();
      DiscretizationTimestep = MeasuredTime - CurrentTime;
      CurrentTime = MeasuredTime;
    }    
    return DiscretizationTimestep;
  }

  /** 
   * Closes this instance and all held resources immediately 
   */
  public synchronized void close() {
    
  }

  /**
   * Calculates the force of drag on a real world object according to the equation:
   * <pre><code>
   * F = 1/2 * p * v^2 * a * c 
   * </code></pre>
   * With the density of air 1.1839                            (Kg/m^3)
   * @param Velocity Relative velocity of the object in motion (m/s)
   * @param Area     Cross-sectional area of a given object    (m^2)
   * @param Mu       Friction coefficient of the object        (None)
   * @return The force, in Newtons of the object               (N)
   */
  public Double drag(final Double Velocity, final Double Area, final Double Mu) {
    return (1/2) * Measurements.EARTH_AIR_DENSITY * Math.pow(Velocity,(2)) * Mu * Area;
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
