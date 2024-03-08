// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.lib.motion.actuators.archetype;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import org.littletonrobotics.junction.Logger;
import org.robotalons.lib.motion.actuators.Module;

import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.stream.IntStream;
// --------------------------------------------------------------[Sim Module]-------------------------------------------------------------//
/**
 *
 *
 * <h1>SimModule</h1>
 *
 * <p>Implementation of a single swerve module unit which utilizes simulated Linear Motor Systems as hardware.</p>
 * 
 * @see Module
 * 
 * @author Cody Washington (@Jelatinone) 
 */
public class SimModule<Controller extends DCMotorSim> extends Module {
  // --------------------------------------------------------------[Constants]--------------------------------------------------------------//
  private final Controller TRANSLATIONAL_CONTROLLER;
  private final SimpleMotorFeedforward TRANSLATIONAL_FF;
  private final PIDController TRANSLATIONAL_PID;
  private final Queue<Double> TRANSLATIONAL_POSITION_QUEUE;

  private final Controller ROTATIONAL_CONTROLLER;
  private final PIDController ROTATIONAL_PID;
  private final Queue<Double> ROTATIONAL_POSITION_QUEUE;

  private final Queue<Double> TIMESTAMP_QUEUE;
  private final List<SwerveModulePosition> DELTAS;
  private final List<Double> TIMESTAMPS;
  private final Lock ODOMETRY_LOCK;

  private final ModuleConfiguration<Controller> MODULE_CONSTANTS;
  // ---------------------------------------------------------------[Fields]----------------------------------------------------------------//
  private Double TranslationalIntegratedPosition;
  private Double RotationalIntegratedPosition;
  private Double DiscretizationPreviousTimestamp;
  // -----------------------------------------------------------[Constructor(s)]------------------------------------------------------------//
  /**
   * Spark Module Constructor
   * @param Constants Constant values to construct a new module from
   */
  public SimModule(final ModuleConfiguration<Controller> Constants) {
    super(Constants);
    MODULE_CONSTANTS = Constants;
    TRANSLATIONAL_CONTROLLER = (Controller) MODULE_CONSTANTS.TRANSLATIONAL_CONTROLLER;
    TRANSLATIONAL_PID = new PIDController(
      MODULE_CONSTANTS.TRANSLATIONAL_PID_CONSTANTS.kP,
      MODULE_CONSTANTS.TRANSLATIONAL_PID_CONSTANTS.kI, 
      MODULE_CONSTANTS.TRANSLATIONAL_PID_CONSTANTS.kD);
    TRANSLATIONAL_FF = new SimpleMotorFeedforward(
      MODULE_CONSTANTS.TRANSLATIONAL_KS_GAIN,
      MODULE_CONSTANTS.TRANSLATIONAL_KV_GAIN,
      MODULE_CONSTANTS.TRANSLATIONAL_KA_GAIN);
    ROTATIONAL_CONTROLLER = (Controller) MODULE_CONSTANTS.ROTATIONAL_CONTROLLER;
    ROTATIONAL_PID = new PIDController(
      MODULE_CONSTANTS.ROTATIONAL_PID_CONSTANTS.kP,
      MODULE_CONSTANTS.ROTATIONAL_PID_CONSTANTS.kI, 
      MODULE_CONSTANTS.ROTATIONAL_PID_CONSTANTS.kD);
    
    TRANSLATIONAL_POSITION_QUEUE = MODULE_CONSTANTS.STATUS_PROVIDER.register(() -> TranslationalIntegratedPosition);
    ROTATIONAL_POSITION_QUEUE = MODULE_CONSTANTS.STATUS_PROVIDER.register(() -> RotationalIntegratedPosition);
    TIMESTAMP_QUEUE = MODULE_CONSTANTS.STATUS_PROVIDER.timestamp();
    ODOMETRY_LOCK = new ReentrantLock();

    RotationalAbsoluteOffset = MODULE_CONSTANTS.ROTATIONAL_ENCODER_OFFSET;
    RotationalIntegratedPosition = new Rotation2d(Math.random() * 2d * Math.PI).getRadians();
    TranslationalIntegratedPosition = new Rotation2d(Math.random() * 2d * Math.PI).getRadians();

    TIMESTAMPS = new ArrayList<>();
    DELTAS = new ArrayList<>();

    DiscretizationPreviousTimestamp = Logger.getRealTimestamp() / (1e6);

    configure();
  }
  // ---------------------------------------------------------------[Methods]---------------------------------------------------------------//
  /**
   * Force configures this module's hardware to standard SparkModule specifications.
   */
  private synchronized void configure() {
    ODOMETRY_LOCK.lock();
    cease();
    ROTATIONAL_PID.enableContinuousInput(-Math.PI, Math.PI);
    ODOMETRY_LOCK.unlock();
  }

  @Override
  public synchronized void close() {
    ReferenceMode = ReferenceType.CLOSED;
    TRANSLATIONAL_POSITION_QUEUE.clear();
    ROTATIONAL_POSITION_QUEUE.clear();
    TIMESTAMP_QUEUE.clear();
    TIMESTAMPS.clear();
    DELTAS.clear();
  }

  @Override
  public synchronized void cease() {
    TRANSLATIONAL_CONTROLLER.setInputVoltage((0d));
    ROTATIONAL_CONTROLLER.setInputVoltage((0d));
  }

  @Override
  public synchronized void periodic() {
    final var Timestamp = discretize();
    ROTATIONAL_CONTROLLER.update(Timestamp);
    TRANSLATIONAL_CONTROLLER.update(Timestamp);
    RotationalIntegratedPosition += ROTATIONAL_CONTROLLER.getAngularVelocityRadPerSec() * (Timestamp);
    TranslationalIntegratedPosition += TRANSLATIONAL_CONTROLLER.getAngularVelocityRadPerSec() * (Timestamp) / MODULE_CONSTANTS.WHEEL_RADIUS_METERS * Math.cos(RotationalIntegratedPosition);
    update();
    synchronized(STATUS) {
      if (RotationalRelativeOffset == (null) && STATUS.RotationalAbsolutePosition.getRadians() != (0d)) {
        RotationalRelativeOffset = STATUS.RotationalAbsolutePosition.minus(STATUS.RotationalRelativePosition);
      }
      switch(ReferenceMode) {
        case STATE_CONTROL:
          if(Reference != (null)) {
            if (Reference.angle != (null)) {
              setRotationalVoltage(ROTATIONAL_PID.calculate(getAbsoluteRotation().getRadians(),Reference.angle.getRadians()));
            }
            var Adjusted = (Reference.speedMetersPerSecond * Math.cos(ROTATIONAL_PID.getPositionError())) / MODULE_CONSTANTS.WHEEL_RADIUS_METERS;
            setTranslationalVoltage(-(TRANSLATIONAL_PID.calculate(Adjusted)) + (TRANSLATIONAL_FF.calculate(STATUS.TranslationalVelocityRadiansSecond, Adjusted)));          
          }
          break;
        case DISABLED:
          cease();
          break;
        case CLOSED:
          close();
          break;
      }
      synchronized(DELTAS) {
        DELTAS.clear();
        IntStream.range((0), Math.min(STATUS.OdometryTranslationalPositionsRadians.length, STATUS.OdometryRotationalPositionsRadians.length)).forEachOrdered((Index) -> {
          DELTAS.add(new SwerveModulePosition(
            STATUS.OdometryTranslationalPositionsRadians[Index] * MODULE_CONSTANTS.WHEEL_RADIUS_METERS,
            STATUS.OdometryRotationalPositionsRadians[Index]
          ));
        });    
      }
    }
  }

  /**
   * Calculates the discretization timestep, {@code dt}, at this current time based on the FPGA clock.
   * @return Double representation of the time passed between now and the last timestep.
   */
  private synchronized Double discretize() {
    var DiscretizationTimestep = (0.0);
    if (DiscretizationPreviousTimestamp.equals((0.0))) {
      DiscretizationTimestep = ((1.0) / (50.0));
    } else {
      var MeasuredTime = Logger.getRealTimestamp() / (1e6);
      DiscretizationTimestep = MeasuredTime - DiscretizationPreviousTimestamp;
      DiscretizationPreviousTimestamp = MeasuredTime;
    }    
    return DiscretizationTimestep;
  }
  
  /**
   * Forces an update of the AutoLogged values to AdvantageKit, logged with the corresponding module number
   */
  @Override
  public synchronized void update() {
    ODOMETRY_LOCK.lock();
    MODULE_CONSTANTS.STATUS_PROVIDER.getLock().lock();
    synchronized(STATUS) {
      STATUS.TranslationalPositionRadians = TRANSLATIONAL_CONTROLLER.getAngularPositionRad();
      STATUS.TranslationalVelocityRadiansSecond = TRANSLATIONAL_CONTROLLER.getAngularPositionRad();
      STATUS.TranslationalCurrentAmperage = TRANSLATIONAL_CONTROLLER.getCurrentDrawAmps();

      STATUS.RotationalVelocityRadiansSecond = ROTATIONAL_CONTROLLER.getAngularVelocityRadPerSec();
      STATUS.RotationalAppliedAmperage = ROTATIONAL_CONTROLLER.getCurrentDrawAmps();      
      STATUS.RotationalRelativePosition = new Rotation2d(RotationalIntegratedPosition);
      STATUS.RotationalAbsolutePosition = new Rotation2d(RotationalIntegratedPosition);
    
      synchronized(TIMESTAMP_QUEUE) {
        TIMESTAMPS.clear();
        STATUS.OdometryTimestamps = 
          TIMESTAMP_QUEUE.stream()
            .mapToDouble((final Double Value) -> {
              TIMESTAMPS.add(Value);
              return Value.doubleValue();
            }).toArray();
        TIMESTAMP_QUEUE.clear();          
      }
      synchronized(TRANSLATIONAL_POSITION_QUEUE) {
        STATUS.OdometryTranslationalPositionsRadians =
          TRANSLATIONAL_POSITION_QUEUE.stream()
            .mapToDouble((final Double Value) -> Units.rotationsToRadians(Value) / MODULE_CONSTANTS.ROTATIONAL_GEAR_RATIO)
            .toArray();    
        TRANSLATIONAL_POSITION_QUEUE.clear();    
      }
      synchronized(ROTATIONAL_POSITION_QUEUE) {
        STATUS.OdometryRotationalPositionsRadians =
          ROTATIONAL_POSITION_QUEUE.stream()
            .map((final Double Value) -> Rotation2d.fromRotations(Value / MODULE_CONSTANTS.TRANSLATIONAL_GEAR_RATIO))
            .toArray(Rotation2d[]::new);    
        ROTATIONAL_POSITION_QUEUE.clear();  
      }
    }
    Logger.processInputs("RealInputs/" + "MODULE (" + Integer.toString(MODULE_CONSTANTS.NUMBER) + ')', STATUS);
    MODULE_CONSTANTS.STATUS_PROVIDER.getLock().unlock();
    ODOMETRY_LOCK.unlock();
  }

  /**
   * Zeroes the rotational relative to offset from the position of the absolute encoders.
   */
  @Override
  public synchronized void reset() {
    update();
  }

  @Override
  protected synchronized void setTranslationalVoltage(final Double Voltage) {
    TRANSLATIONAL_CONTROLLER.setInputVoltage(MathUtil.clamp(Voltage != null? Voltage: 0d, (-12d), (12d)));
  }

  @Override
  protected synchronized void setRotationalVoltage(final Double Voltage) {
    ROTATIONAL_CONTROLLER.setInputVoltage(MathUtil.clamp(Voltage != null? Voltage: 0d, (-12d), (12d)));
  }
  

  // --------------------------------------------------------------[Mutators]---------------------------------------------------------------//
  @Override
  public SwerveModuleState set(final SwerveModuleState Reference) {
    this.Reference = Reference;
    return this.Reference;
  }
  // --------------------------------------------------------------[Accessors]--------------------------------------------------------------//
  @Override
  public List<SwerveModulePosition> getPositionDeltas() {
    return DELTAS;
  }

  @Override
  public List<Double> getPositionTimestamps() {
    return TIMESTAMPS;
  }
}
