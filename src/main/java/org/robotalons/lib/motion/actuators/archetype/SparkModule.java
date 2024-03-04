// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.lib.motion.actuators.archetype;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.RelativeEncoder;

import org.littletonrobotics.junction.Logger;
import org.robotalons.lib.motion.actuators.Module;

import java.util.ArrayList;
import java.util.ConcurrentModificationException;
import java.util.List;
import java.util.Queue;
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
 */
public class SparkModule<Controller extends CANSparkMax> extends Module {
  // --------------------------------------------------------------[MODULE_CONSTANTS]--------------------------------------------------------------//
  private final Controller TRANSLATIONAL_CONTROLLER;
  private final PIDController TRANSLATIONAL_PID;
  private final SimpleMotorFeedforward TRANSLATIONAL_FF;
  private final RelativeEncoder TRANSLATIONAL_ENCODER;
  private final Queue<Double> TRANSLATIONAL_VELOCITY_QUEUE;

  private final Controller ROTATIONAL_CONTROLLER;
  private final PIDController ROTATIONAL_PID;
  private final RelativeEncoder ROTATIONAL_ENCODER;
  private final Queue<Double> ROTATIONAL_POSITION_QUEUE;   

  private final CANcoder ABSOLUTE_ENCODER;
  private final Queue<Double> TIMESTAMP_QUEUE;
  private final List<SwerveModulePosition> DELTAS;
  private final List<Double> TIMESTAMPS;
  private final Lock ODOMETRY_LOCK;

  private final ModuleConfiguration<Controller> MODULE_CONSTANTS;
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
    ROTATIONAL_CONTROLLER = MODULE_CONSTANTS.ROTATIONAL_CONTROLLER;
    ROTATIONAL_PID = new PIDController(
      MODULE_CONSTANTS.ROTATIONAL_PID_CONSTANTS.kP, 
      MODULE_CONSTANTS.ROTATIONAL_PID_CONSTANTS.kI, 
      MODULE_CONSTANTS.ROTATIONAL_PID_CONSTANTS.kD);
    ROTATIONAL_ENCODER = ROTATIONAL_CONTROLLER.getEncoder();
    ABSOLUTE_ENCODER = new CANcoder(MODULE_CONSTANTS.ABSOLUTE_ENCODER_PORT);

    RotationalAbsoluteOffset = MODULE_CONSTANTS.ROTATIONAL_ENCODER_OFFSET;
    ODOMETRY_LOCK = new ReentrantLock();

    TRANSLATIONAL_VELOCITY_QUEUE = MODULE_CONSTANTS.STATUS_PROVIDER.register(TRANSLATIONAL_ENCODER::getVelocity);
    ROTATIONAL_POSITION_QUEUE = MODULE_CONSTANTS.STATUS_PROVIDER.register(ROTATIONAL_ENCODER::getPosition);
    TIMESTAMP_QUEUE = MODULE_CONSTANTS.STATUS_PROVIDER.timestamp();
    
    TIMESTAMPS = new ArrayList<>();
    DELTAS = new ArrayList<>();

    configure();
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

    ROTATIONAL_ENCODER.setPosition(
      (RobotBase.isReal())?
        (Rotation2d.fromRotations(ABSOLUTE_ENCODER.getAbsolutePosition().getValueAsDouble()).minus(RotationalAbsoluteOffset).getRotations() * MODULE_CONSTANTS.ROTATIONAL_GEAR_RATIO):
        (0d)
    );
    ROTATIONAL_ENCODER.setAverageDepth((2));
    ROTATIONAL_ENCODER.setMeasurementPeriod((10));

    ROTATIONAL_PID.enableContinuousInput(-Math.PI, Math.PI);

    IntStream.range((0),(4)).forEach((Index) -> {
      TRANSLATIONAL_CONTROLLER.setPeriodicFramePeriod(
          PeriodicFrame.kStatus2, (int) (1000d / org.robotalons.crescendo.subsystems.drivebase.Constants.Measurements.ODOMETRY_FREQUENCY));
      ROTATIONAL_CONTROLLER.setPeriodicFramePeriod(
          PeriodicFrame.kStatus2, (int) (1000d / org.robotalons.crescendo.subsystems.drivebase.Constants.Measurements.ODOMETRY_FREQUENCY));      
    });

    TRANSLATIONAL_CONTROLLER.setCANTimeout((0));
    ROTATIONAL_CONTROLLER.setCANTimeout((0));

    TRANSLATIONAL_CONTROLLER.burnFlash();
    ROTATIONAL_CONTROLLER.burnFlash();
    ODOMETRY_LOCK.unlock();
  }

  @Override
  public synchronized void close() {
    ReferenceMode = ReferenceType.CLOSED;
    TRANSLATIONAL_CONTROLLER.close();
    ROTATIONAL_CONTROLLER.close();
    ABSOLUTE_ENCODER.close();
    TRANSLATIONAL_VELOCITY_QUEUE.clear();
    ROTATIONAL_POSITION_QUEUE.clear();
    TIMESTAMP_QUEUE.clear();
    TIMESTAMPS.clear();
    DELTAS.clear();
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
    ROTATIONAL_ENCODER.setPosition(
      RobotBase.isReal()?
      Rotation2d.fromRotations(ABSOLUTE_ENCODER.getAbsolutePosition().getValueAsDouble()).minus(RotationalAbsoluteOffset).getRotations() * MODULE_CONSTANTS.ROTATIONAL_GEAR_RATIO:
      (0d)
    );
  }

  @Override
  public synchronized void periodic() {
    update();
    synchronized(STATUS) {
      if (RotationalRelativeOffset == (null) && STATUS.RotationalAbsolutePosition.getRadians() != (0d)) {
        RotationalRelativeOffset = STATUS.RotationalAbsolutePosition.minus(STATUS.RotationalRelativePosition);
      }
      switch(ReferenceMode) {
        case STATE_CONTROL:
          if(Reference != (null)) {
            if (Reference.angle != (null)) {
              setRotationalVoltage(ROTATIONAL_PID.calculate(getRelativeRotation().getRadians(),Reference.angle.getRadians() * MODULE_CONSTANTS.ROTATIONAL_GEAR_RATIO));
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
        IntStream.range((0), STATUS.OdometryTimestamps.length).parallel().forEach((Index) -> {
          final var Rotation = STATUS.OdometryRotationalPositions[Index].plus((RotationalRelativeOffset != null)? (RotationalRelativeOffset): (new Rotation2d()));
          final var Position = STATUS.OdometryTranslationalPositionsRadians[Index] * MODULE_CONSTANTS.WHEEL_RADIUS_METERS;
          DELTAS.add(new SwerveModulePosition(Position, Rotation));
        });
      }
    }
  }


  @Override
  protected synchronized void setTranslationalVoltage(final Double Voltage) {
    TRANSLATIONAL_CONTROLLER.setVoltage(MathUtil.clamp(Voltage, (-12d), (12d)));
  }

  @Override
  protected synchronized void setRotationalVoltage(final Double Voltage) {
    ROTATIONAL_CONTROLLER.setVoltage(MathUtil.clamp(Voltage, (-12d), (12d)));
  }
  
  /**
   * Forces an update of the AutoLogged values to AdvantageKit, logged with the corresponding module number
   */
  @Override
  public synchronized void update() {
    ODOMETRY_LOCK.lock();
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
          Rotation2d.fromRotations(ABSOLUTE_ENCODER.getAbsolutePosition().getValue()).minus(RotationalAbsoluteOffset);
        STATUS.RotationalRelativePosition =
          Rotation2d.fromRotations(ROTATIONAL_ENCODER.getPosition() / MODULE_CONSTANTS.ROTATIONAL_GEAR_RATIO);
        STATUS.RotationalVelocityRadiansSecond =
            Units.rotationsPerMinuteToRadiansPerSecond(ROTATIONAL_ENCODER.getVelocity()) / MODULE_CONSTANTS.ROTATIONAL_GEAR_RATIO;
        STATUS.RotationalAppliedVoltage = 
          MODULE_CONSTANTS.ROTATIONAL_CONTROLLER.getAppliedOutput() * MODULE_CONSTANTS.ROTATIONAL_CONTROLLER.getBusVoltage();
        STATUS.RotationalAppliedAmperage = MODULE_CONSTANTS.ROTATIONAL_CONTROLLER.getOutputCurrent();
        STATUS.RotationalTemperatureCelsius = 
          ROTATIONAL_CONTROLLER.getMotorTemperature();
      try {
        synchronized(TIMESTAMP_QUEUE) {
          synchronized(STATUS.OdometryTimestamps) {
            STATUS.OdometryTimestamps = 
              TIMESTAMP_QUEUE.stream()
                .mapToDouble(Double::doubleValue).toArray();
            TIMESTAMP_QUEUE.clear();          
          }
        }
        synchronized(TRANSLATIONAL_VELOCITY_QUEUE) {
          synchronized(STATUS.OdometryTranslationalPositionsRadians) {
            STATUS.OdometryTranslationalPositionsRadians =
              TRANSLATIONAL_VELOCITY_QUEUE.stream()
                .mapToDouble((Double value) -> Units.rotationsToRadians(value) / MODULE_CONSTANTS.ROTATIONAL_GEAR_RATIO)
                .toArray();    
            TRANSLATIONAL_VELOCITY_QUEUE.clear();    
          }
        }
        synchronized(ROTATIONAL_POSITION_QUEUE) {
          synchronized(STATUS.OdometryRotationalPositions) {
            STATUS.OdometryRotationalPositions =
              ROTATIONAL_POSITION_QUEUE.stream()
                .map((Double value) -> Rotation2d.fromRotations(value / MODULE_CONSTANTS.TRANSLATIONAL_GEAR_RATIO))
                .toArray(Rotation2d[]::new);    
            ROTATIONAL_POSITION_QUEUE.clear();  
          }
        }
      } catch (final ConcurrentModificationException Ignored) {}
    }
    Logger.processInputs("RealInputs/" + "MODULE (" + Integer.toString(MODULE_CONSTANTS.NUMBER) + ')', STATUS);
    ODOMETRY_LOCK.unlock();
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
