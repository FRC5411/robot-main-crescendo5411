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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

import org.littletonrobotics.junction.Logger;
import org.robotalons.lib.motion.actuators.Module;

import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.stream.IntStream;
// ------------------------------------------------------------[Flywheel Module]-------------------------------------------------------------//
/**
 *
 *
 * <h1>FlywheelModule</h1>
 *
 * <p>Implementation of a single swerve module unit which utilizes simulated Linear Flywheel Systems as hardware.</p>
 * 
 * @see Module
 */
public class FlywheelModule<Controller extends FlywheelSim> extends Module {
  // --------------------------------------------------------------[Constants]--------------------------------------------------------------//
  private final Controller TRANSLATIONAL_CONTROLLER;
  private final SimpleMotorFeedforward TRANSLATIONAL_FF;
  private final PIDController TRANSLATIONAL_PID;
  private final Queue<Double> TRANSLATIONAL_VELOCITY_QUEUE;

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
  public FlywheelModule(final ModuleConfiguration<Controller> Constants) {
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
    
    TRANSLATIONAL_VELOCITY_QUEUE = MODULE_CONSTANTS.STATUS_PROVIDER.register(TRANSLATIONAL_CONTROLLER::getAngularVelocityRPM);
    ROTATIONAL_POSITION_QUEUE = MODULE_CONSTANTS.STATUS_PROVIDER.register(ROTATIONAL_CONTROLLER::getAngularVelocityRadPerSec);
    TIMESTAMP_QUEUE = MODULE_CONSTANTS.STATUS_PROVIDER.timestamp();
    ODOMETRY_LOCK = new ReentrantLock();

    RotationalIntegratedPosition = (MODULE_CONSTANTS.TRANSLATIONAL_POSITION_METERS);
    TranslationalIntegratedPosition = (MODULE_CONSTANTS.ROTATIONAL_ENCODER_OFFSET.getRadians());

    TIMESTAMPS = new ArrayList<>();
    DELTAS = new ArrayList<>();

    RotationalAbsoluteOffset = MODULE_CONSTANTS.ROTATIONAL_ENCODER_OFFSET;

    DiscretizationPreviousTimestamp = Timer.getFPGATimestamp();

    configure();
  }
  // ---------------------------------------------------------------[Methods]---------------------------------------------------------------//
  /**
   * Force configures this module's hardware to standard SparkModule specifications.
   */
  private synchronized void configure() {
    cease();
    ODOMETRY_LOCK.lock();
    TRANSLATIONAL_PID.enableContinuousInput((-1), (1));
    ROTATIONAL_PID.enableContinuousInput(-Math.PI, Math.PI);
    ODOMETRY_LOCK.unlock();
  }

  @Override
  public synchronized void close() {
    ReferenceMode = ReferenceType.CLOSED;
    TRANSLATIONAL_VELOCITY_QUEUE.clear();
    ROTATIONAL_POSITION_QUEUE.clear();
    TIMESTAMP_QUEUE.clear();
    TIMESTAMPS.clear();
    DELTAS.clear();
  }

  @Override
  public synchronized void cease() {
    ROTATIONAL_CONTROLLER.setInputVoltage((0d));
    TRANSLATIONAL_CONTROLLER.setInputVoltage((0d));
  }

  @Override
  public synchronized void periodic() {
    final var DiscretizationTimestamp = discretize();
    ROTATIONAL_CONTROLLER.update(DiscretizationTimestamp);
    TRANSLATIONAL_CONTROLLER.update(DiscretizationTimestamp);
    RotationalIntegratedPosition += ROTATIONAL_CONTROLLER.getAngularVelocityRPM() * (DiscretizationTimestamp);
    TranslationalIntegratedPosition += TRANSLATIONAL_CONTROLLER.getAngularVelocityRPM() * (DiscretizationTimestamp);
    ODOMETRY_LOCK.lock();
    update();
    if (RotationalRelativeOffset == (null) && Status.RotationalAbsolutePosition.getRadians() != (0d)) {
      RotationalRelativeOffset = Status.RotationalAbsolutePosition.minus(Status.RotationalRelativePosition);
    }
    switch(ReferenceMode) {
      case STATE_CONTROL:
      if(Reference != (null)) {
        if (Reference.angle != (null)) {
          setRotationalVoltage(ROTATIONAL_PID.calculate(getRelativeRotation().getRadians(),Reference.angle.getRadians()));
        }
        var AdjustReferenceVelocity = (Reference.speedMetersPerSecond * Math.cos(ROTATIONAL_PID.getPositionError())) / MODULE_CONSTANTS.WHEEL_RADIUS_METERS;
        setTranslationalVoltage(     -(TRANSLATIONAL_PID.calculate(AdjustReferenceVelocity))
                                                            +
          (TRANSLATIONAL_FF.calculate(Status.TranslationalVelocityRadiansSecond, AdjustReferenceVelocity)));          
      }
        break;
      case DISABLED:
        cease();
        break;
      case CLOSED:
        close();
        break;
    }
    DELTAS.clear();
    IntStream.range((0), (Status.OdometryTimestamps.length - 1)).forEach((Index) -> {
      final var Position = Status.OdometryTranslationalPositionsRadians[Index] * CONSTANTS.WHEEL_RADIUS_METERS;
      final var Rotation = Status.OdometryRotationalPositions[Index].plus(
        RotationalRelativeOffset != null? RotationalRelativeOffset: new Rotation2d());
      DELTAS.add(new SwerveModulePosition(Position, Rotation));
    });
    ODOMETRY_LOCK.unlock();  

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
      var MeasuredTime = Timer.getFPGATimestamp();
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
    Status.TranslationalPositionRadians = TranslationalIntegratedPosition;
    Status.TranslationalVelocityRadiansSecond =
        Units.rotationsPerMinuteToRadiansPerSecond(TRANSLATIONAL_CONTROLLER.getAngularVelocityRPM()) / CONSTANTS.TRANSLATIONAL_GEAR_RATIO;
    Status.RotationalRelativePosition = new Rotation2d(RotationalIntegratedPosition);
    Status.RotationalAbsolutePosition = new Rotation2d(RotationalIntegratedPosition);
    Status.TranslationalAppliedVoltage = 
      TRANSLATIONAL_CONTROLLER.getCurrentDrawAmps() * (5);
    Status.TranslationalCurrentAmperage = TRANSLATIONAL_CONTROLLER.getCurrentDrawAmps();
    Status.RotationalVelocityRadiansSecond =
        Units.rotationsPerMinuteToRadiansPerSecond(ROTATIONAL_CONTROLLER.getAngularVelocityRPM()) / CONSTANTS.ROTATIONAL_GEAR_RATIO;
    Status.RotationalAppliedAmperage = ROTATIONAL_CONTROLLER.getCurrentDrawAmps();
    Status.OdometryTimestamps = TIMESTAMP_QUEUE.stream().mapToDouble(Double::valueOf).toArray();
    MODULE_CONSTANTS.STATUS_PROVIDER.getLock().lock();
    Status.OdometryTranslationalPositionsRadians =
      TRANSLATIONAL_VELOCITY_QUEUE.stream()
        .mapToDouble((Double value) -> Units.rotationsToRadians(value) / CONSTANTS.ROTATIONAL_GEAR_RATIO)
        .toArray();
    Status.OdometryRotationalPositions =
      ROTATIONAL_POSITION_QUEUE.stream()
        .map((Double value) -> Rotation2d.fromRotations(value / CONSTANTS.TRANSLATIONAL_GEAR_RATIO))
        .toArray(Rotation2d[]::new);
    MODULE_CONSTANTS.STATUS_PROVIDER.getLock().unlock();
    TRANSLATIONAL_VELOCITY_QUEUE.clear();
    ROTATIONAL_POSITION_QUEUE.clear();
    TIMESTAMP_QUEUE.clear();
    Logger.processInputs("RealInputs/" + "MODULE (" + Integer.toString(CONSTANTS.NUMBER) + ')', Status);
  }

  /**
   * Zeroes the rotational relative to offset from the position of the absolute encoders.
   */
  @Override
  public synchronized void reset() {
    update();
    RotationalIntegratedPosition = (0d);
  }

  @Override
  protected synchronized void setTranslationalVoltage(final Double Voltage) {
    TRANSLATIONAL_CONTROLLER.setInputVoltage(MathUtil.clamp(Voltage, (-12d), (12d)));
  }

  @Override
  protected synchronized void setRotationalVoltage(final Double Voltage) {
    ROTATIONAL_CONTROLLER.setInputVoltage(MathUtil.clamp(Voltage, (-12d), (12d)));
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
