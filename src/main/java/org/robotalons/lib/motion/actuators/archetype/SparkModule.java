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

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import org.littletonrobotics.junction.Logger;
import org.robotalons.lib.motion.actuators.Module;

import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.atomic.DoubleAccumulator;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.stream.IntStream;
// --------------------------------------------------------------[Spark Module]-------------------------------------------------------------//
/**
 *
 *
 * <h1>REVControllerModule</h1>
 *
 * <p>Implementation of a single swerve module unit which utilizes REV Controllers (SparkMax) as hardware.</p>
 * 
 * @see Module
 * 
 * @author Cody Washington (@Jelatinone) 
 */
public class SparkModule<Controller extends CANSparkMax> extends Module {
  // --------------------------------------------------------------[Constants]----------------------------------------------------------------//
  private static final Long ABSOLUTE_ENCODER_WAIT_TIMEOUT = (1000L);

  private final Controller TRANSLATIONAL_CONTROLLER;
  private final PIDController TRANSLATIONAL_PID;
  private final SimpleMotorFeedforward TRANSLATIONAL_FF;
  private final RelativeEncoder TRANSLATIONAL_ENCODER;
  private final Queue<Double> TRANSLATIONAL_POSITION_QUEUE;
  private final DoubleAccumulator TRANSLATIONAL_POSITION;

  private final Controller ROTATIONAL_CONTROLLER;
  private final PIDController ROTATIONAL_PID;
  private final RelativeEncoder ROTATIONAL_ENCODER;
  private final Queue<Double> ROTATIONAL_POSITION_QUEUE;   
  private final CANcoder ABSOLUTE_ENCODER;

  private final List<SwerveModulePosition> POSITION_DELTAS;

  private final Queue<Double> TIMESTAMP_QUEUE;
  private final List<Double> TIMESTAMPS;

  private final Lock ODOMETRY_LOCK;

  private final ModuleConfiguration<Controller> MODULE_CONSTANTS;
  // ---------------------------------------------------------------[Fields]----------------------------------------------------------------//
  private volatile Double Timestamp = Logger.getRealTimestamp() / (1e6);
  // -----------------------------------------------------------[Constructor(s)]------------------------------------------------------------//
  /**
   * Spark Module Constructor
   * @param Constants Constant values to construct a new module from
   */
  public SparkModule(final ModuleConfiguration<Controller> Constants) {
    super(Constants);
    this.MODULE_CONSTANTS = Constants;
    TRANSLATIONAL_CONTROLLER = MODULE_CONSTANTS.TRANSLATIONAL_CONTROLLER;
    TRANSLATIONAL_PID = new PIDController(
      MODULE_CONSTANTS.TRANSLATIONAL_PID_CONSTANTS.kP, 
      MODULE_CONSTANTS.TRANSLATIONAL_PID_CONSTANTS.kI, 
      MODULE_CONSTANTS.TRANSLATIONAL_PID_CONSTANTS.kD);
    TRANSLATIONAL_FF = new SimpleMotorFeedforward(
      MODULE_CONSTANTS.TRANSLATIONAL_KS_GAIN,
      MODULE_CONSTANTS.TRANSLATIONAL_KV_GAIN,
      MODULE_CONSTANTS.TRANSLATIONAL_KA_GAIN);
    TRANSLATIONAL_ENCODER = TRANSLATIONAL_CONTROLLER.getEncoder();

    TRANSLATIONAL_POSITION = new DoubleAccumulator(
      (Accumulated, Velocity) -> Accumulated -= Units.rotationsPerMinuteToRadiansPerSecond(Velocity) * discretize() * MODULE_CONSTANTS.WHEEL_RADIUS_METERS, (0d));

    ROTATIONAL_CONTROLLER = MODULE_CONSTANTS.ROTATIONAL_CONTROLLER;
    ROTATIONAL_PID = new PIDController(
      MODULE_CONSTANTS.ROTATIONAL_PID_CONSTANTS.kP, 
      MODULE_CONSTANTS.ROTATIONAL_PID_CONSTANTS.kI, 
      MODULE_CONSTANTS.ROTATIONAL_PID_CONSTANTS.kD);
    ROTATIONAL_ENCODER = ROTATIONAL_CONTROLLER.getEncoder();
    ABSOLUTE_ENCODER = new CANcoder(MODULE_CONSTANTS.ABSOLUTE_ENCODER_PORT);

    try {
      wait(ABSOLUTE_ENCODER_WAIT_TIMEOUT);
    } catch (final InterruptedException Ignored) {}

    RotationalAbsoluteOffset = MODULE_CONSTANTS.ROTATIONAL_ENCODER_OFFSET;
    ODOMETRY_LOCK = new ReentrantLock();

    TRANSLATIONAL_POSITION_QUEUE = MODULE_CONSTANTS.STATUS_PROVIDER.register(TRANSLATIONAL_POSITION::get);
    ROTATIONAL_POSITION_QUEUE = MODULE_CONSTANTS.STATUS_PROVIDER.register(() -> getAbsoluteRotation().getRadians());
    TIMESTAMP_QUEUE = MODULE_CONSTANTS.STATUS_PROVIDER.timestamp();
    
    TIMESTAMPS = new ArrayList<>();
    POSITION_DELTAS = new ArrayList<>();

    BaseStatusSignal.setUpdateFrequencyForAll((25), ABSOLUTE_ENCODER.getAbsolutePosition());
    ABSOLUTE_ENCODER.optimizeBusUtilization();

    configure();
    reset();
  }
  // ---------------------------------------------------------------[Methods]---------------------------------------------------------------//
  /**
   * Force configures this module's hardware to standard SparkModule specifications.
   */
  private synchronized void configure() {
    cease();
    ODOMETRY_LOCK.lock();

    TRANSLATIONAL_CONTROLLER.restoreFactoryDefaults();
    ROTATIONAL_CONTROLLER.restoreFactoryDefaults();

    TRANSLATIONAL_CONTROLLER.setCANTimeout((250));
    ROTATIONAL_CONTROLLER.setCANTimeout((250));

    ROTATIONAL_CONTROLLER.setInverted(MODULE_CONSTANTS.ROTATIONAL_INVERTED);
    TRANSLATIONAL_CONTROLLER.setInverted(MODULE_CONSTANTS.TRANSLATIONAL_INVERTED);
    TRANSLATIONAL_CONTROLLER.setIdleMode(IdleMode.kBrake);
    ROTATIONAL_CONTROLLER.setIdleMode(IdleMode.kCoast);

    ROTATIONAL_CONTROLLER.setSmartCurrentLimit((30));
    TRANSLATIONAL_CONTROLLER.setSmartCurrentLimit((40));
    
    TRANSLATIONAL_CONTROLLER.enableVoltageCompensation((12d));
    ROTATIONAL_CONTROLLER.enableVoltageCompensation((12d));

    TRANSLATIONAL_ENCODER.setPosition((0d));
    TRANSLATIONAL_ENCODER.setMeasurementPeriod((10));
    TRANSLATIONAL_ENCODER.setAverageDepth((2));

    ROTATIONAL_ENCODER.setPosition((0d));
    ROTATIONAL_ENCODER.setAverageDepth((2));
    ROTATIONAL_ENCODER.setMeasurementPeriod((10));

    ROTATIONAL_PID.enableContinuousInput(-Math.PI, Math.PI);

    IntStream.range((0),(4)).forEach((Index) -> {
      TRANSLATIONAL_CONTROLLER.setPeriodicFramePeriod(
          PeriodicFrame.kStatus2, (int) (1000d / MODULE_CONSTANTS.STATUS_PROVIDER.getFrequency()));
      ROTATIONAL_CONTROLLER.setPeriodicFramePeriod(
        PeriodicFrame.kStatus2, (int) (1000d / MODULE_CONSTANTS.STATUS_PROVIDER.getFrequency()));      
    });

    TRANSLATIONAL_CONTROLLER.setCANTimeout((0));
    ROTATIONAL_CONTROLLER.setCANTimeout((0));

    TRANSLATIONAL_CONTROLLER.burnFlash();
    ROTATIONAL_CONTROLLER.burnFlash();
    ODOMETRY_LOCK.unlock();

    reset();
  }

  @Override
  public synchronized void close() {
    cease();
    ReferenceMode = ReferenceType.CLOSED;
    TRANSLATIONAL_CONTROLLER.close();
    ROTATIONAL_CONTROLLER.close();
    ABSOLUTE_ENCODER.close();
    TRANSLATIONAL_POSITION_QUEUE.clear();
    ROTATIONAL_POSITION_QUEUE.clear();
    TIMESTAMP_QUEUE.clear();
    TIMESTAMPS.clear();
    POSITION_DELTAS.clear();
  }

  @Override
  public synchronized void cease() {
    TRANSLATIONAL_CONTROLLER.stopMotor();
    ROTATIONAL_CONTROLLER.stopMotor();
  }

  /**
   * Zeroes the azimuth relatively offset from the position of the absolute encoders.
   */
  @Override
  public synchronized void reset() {
    update();
    Reference = new SwerveModuleState();
  }

  /**
   * Calculates the discretization timestep, {@code dt}, at this current time based on the FPGA clock.
   * @return Double representation of the time passed between now and the last timestep.
   */
  private synchronized Double discretize() {
    var Discretized = (0.0);
    if (Timestamp.equals((0d))) {
      Discretized = ((1.0) / (50.0));
    } else {
      final var Measured = Logger.getRealTimestamp() / (1e6);
      Discretized = Measured - Timestamp;
      Timestamp = Measured;
    }    
    return Discretized;
  }

  @Override
  public synchronized void periodic() {
    update();
    TRANSLATIONAL_POSITION.accumulate(TRANSLATIONAL_ENCODER.getVelocity());
    synchronized(STATUS) {
      switch(ReferenceMode) {
        case STATE_CONTROL:
          if(Reference != (null)) {
            if (Reference.angle != (null)) {
              setRotationalVoltage(ROTATIONAL_PID.calculate(getAbsoluteRotation().getRadians(), Reference.angle.getRadians()));
            } else {
              setRotationalVoltage((0d));
            }
            var Adjusted = Reference.speedMetersPerSecond / MODULE_CONSTANTS.WHEEL_RADIUS_METERS;
            setTranslationalVoltage((TRANSLATIONAL_PID.calculate(Adjusted)) + (TRANSLATIONAL_FF.calculate(STATUS.TranslationalVelocityRadiansSecond, Adjusted)));          
          } else {
            cease();
          }
          break;
        case DISABLED:
          cease();
          break;
        case CLOSED:
          close();
          break;
      }
      synchronized(POSITION_DELTAS) {
        POSITION_DELTAS.clear();
        IntStream.range((0), Math.min(STATUS.OdometryTranslationalPositionsRadians.length, STATUS.OdometryRotationalPositionsRadians.length)).parallel().forEach((Index) -> {
          POSITION_DELTAS.add(new SwerveModulePosition(
            STATUS.OdometryTranslationalPositionsRadians[Index] * MODULE_CONSTANTS.WHEEL_RADIUS_METERS,
            STATUS.OdometryRotationalPositionsRadians[Index]
          ));
        });              
      }
    }
  }


  @Override
  protected synchronized void setTranslationalVoltage(final Double Voltage) {
    TRANSLATIONAL_CONTROLLER.setVoltage(MathUtil.clamp(Voltage != null? Voltage: 0d, (-12d), (12d)));
  }

  @Override
  protected synchronized void setRotationalVoltage(final Double Voltage) {
    ROTATIONAL_CONTROLLER.setVoltage(MathUtil.clamp(Voltage != null? Voltage: 0d, (-12d), (12d)));
  }
  
  /**
   * Forces an update of the AutoLogged values to AdvantageKit, logged with the corresponding module number
   */
  @Override
  public synchronized void update() {
    ODOMETRY_LOCK.lock();
    MODULE_CONSTANTS.STATUS_PROVIDER.getLock().lock();
    synchronized(STATUS) {

      STATUS.TranslationalPositionRadians =
        Units.rotationsToRadians(TRANSLATIONAL_ENCODER.getPosition()) / MODULE_CONSTANTS.TRANSLATIONAL_GEAR_RATIO;
      STATUS.TranslationalVelocityRadiansSecond =
          Units.rotationsPerMinuteToRadiansPerSecond(TRANSLATIONAL_ENCODER.getVelocity()) / MODULE_CONSTANTS.TRANSLATIONAL_GEAR_RATIO;
      STATUS.TranslationalAppliedVoltage = 
        MODULE_CONSTANTS.TRANSLATIONAL_CONTROLLER.getAppliedOutput() * MODULE_CONSTANTS.TRANSLATIONAL_CONTROLLER.getBusVoltage();
      STATUS.TranslationalCurrentAmperage = MODULE_CONSTANTS.TRANSLATIONAL_CONTROLLER.getOutputCurrent();
      STATUS.TranslationTemperatureCelsius =
        TRANSLATIONAL_CONTROLLER.getMotorTemperature();

      STATUS.RotationalAbsolutePosition = 
        Rotation2d.fromRotations(ABSOLUTE_ENCODER.getAbsolutePosition().getValueAsDouble()).minus(RotationalAbsoluteOffset);
      STATUS.RotationalRelativePosition =
        Rotation2d.fromRotations(ROTATIONAL_ENCODER.getPosition() / MODULE_CONSTANTS.ROTATIONAL_GEAR_RATIO);
      STATUS.RotationalVelocityRadiansSecond =
          Units.rotationsPerMinuteToRadiansPerSecond(ROTATIONAL_ENCODER.getVelocity()) / MODULE_CONSTANTS.ROTATIONAL_GEAR_RATIO;
      STATUS.RotationalAppliedVoltage = 
        MODULE_CONSTANTS.ROTATIONAL_CONTROLLER.getAppliedOutput() * MODULE_CONSTANTS.ROTATIONAL_CONTROLLER.getBusVoltage();
      STATUS.RotationalCurrentAmperage = MODULE_CONSTANTS.ROTATIONAL_CONTROLLER.getOutputCurrent();
      STATUS.RotationalTemperatureCelsius = 
        ROTATIONAL_CONTROLLER.getMotorTemperature();

      synchronized(TIMESTAMP_QUEUE) {
        synchronized(TIMESTAMPS) {
          TIMESTAMPS.clear();
          STATUS.OdometryTimestamps = 
            TIMESTAMP_QUEUE.stream()
              .mapToDouble((final Double Timestamp) -> {
                TIMESTAMPS.add(Timestamp);
                return Timestamp.doubleValue();
              }).toArray();
          TIMESTAMP_QUEUE.clear();             
        }
      }
      synchronized(TRANSLATIONAL_POSITION_QUEUE) {
        STATUS.OdometryTranslationalPositionsRadians =
          TRANSLATIONAL_POSITION_QUEUE.stream()
            .mapToDouble((final Double Position) -> Units.rotationsToRadians(Position) / MODULE_CONSTANTS.ROTATIONAL_GEAR_RATIO)
            .toArray();    
        TRANSLATIONAL_POSITION_QUEUE.clear();    
      }
      synchronized(ROTATIONAL_POSITION_QUEUE) {
        STATUS.OdometryRotationalPositionsRadians =
          ROTATIONAL_POSITION_QUEUE.stream()
            .map((final Double Position) -> Rotation2d.fromRotations(Position / MODULE_CONSTANTS.TRANSLATIONAL_GEAR_RATIO))
            .toArray(Rotation2d[]::new);    
        ROTATIONAL_POSITION_QUEUE.clear();  
      }
    }
    Logger.processInputs("RealInputs/" + "MODULE (" + Integer.toString(MODULE_CONSTANTS.NUMBER) + ')', STATUS);
    MODULE_CONSTANTS.STATUS_PROVIDER.getLock().unlock();
    ODOMETRY_LOCK.unlock();
    RotationalRelativeOffset = STATUS.RotationalAbsolutePosition.minus(STATUS.RotationalRelativePosition);
  }
  // --------------------------------------------------------------[Accessors]--------------------------------------------------------------//
  @Override
  public List<SwerveModulePosition> getPositionDeltas() {
    synchronized(POSITION_DELTAS) {
      return POSITION_DELTAS; 
    }
  }

  @Override
  public List<Double> getPositionTimestamps() {
    synchronized(TIMESTAMPS) {
      return TIMESTAMPS;
    } 
  }
}