// ----------------------------------------------------------------[Package]---------------------------------------------------------------- //
package org.robotalons.crescendo.subsystems.indexer;

// ---------------------------------------------------------------[Libraries]--------------------------------------------------------------- //
import java.io.Closeable;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.SparkMaxPIDController;

import org.robotalons.crescendo.subsystems.indexer.Constants.Measurements;


// ------------------------------------------------------------[Indexer Subsystem]---------------------------------------------------------- //
/**
 *
 * <h1>IndexerSubsystem</h1>
 *
 * <p>Utility class which controls the indexing of notes to and from the intake and shooter.<p>
 * 
 * @see SubsystemBase
 * @see {@link org.robotalons.crescendo.RobotContainer RobotContainer} 
 */

public class IndexerSubsystem extends SubsystemBase implements Closeable {
    // --------------------------------------------------------------[Constants]-------------------------------------------------------------- //
    private final static CANSparkMax INTAKE_SPARK_MAX;
    private final static CANSparkMax CANNON_SPARK_MAX;
    private final static CANSparkMax[] MOTORS;
    
    private static final Encoder INTAKE_SPARK_MAX_ENCODER;
    private static final Encoder CANNON_SPARK_MAX_ENCODER;
    private static final Encoder[] ENCODERS;
    
    private static final SparkMaxPIDController INTAKE_SPARK_MAX_PID_CONTROLLER;
    private static final SparkMaxPIDController CANNON_SPARK_MAX_PID_CONTROLLER;

    private final static DigitalInput INTAKE_BREAKBEAM_INPUT;
    private final static DigitalInput CANNON_BREAKBEAM_INPUT;

    // ---------------------------------------------------------------[Fields]---------------------------------------------------------------- //
    private static IndexerSubsystem Instance;
    private static Boolean HasNote;

    // ------------------------------------------------------------[Constructors]------------------------------------------------------------- //
    /**
     * Indexer Subsystem Constructor
     */
    public IndexerSubsystem() {} static {
        Instance = new IndexerSubsystem();
        HasNote = (false);

        INTAKE_SPARK_MAX = new CANSparkMax(Constants.Ports.INDEXER_INTAKE_SPARK_MAX_ID, MotorType.kBrushless);
        CANNON_SPARK_MAX = new CANSparkMax(Constants.Ports.INDEXER_CANNON_SPARK_MAX_ID, MotorType.kBrushless);
        MOTORS = new CANSparkMax[]{INTAKE_SPARK_MAX, CANNON_SPARK_MAX};

        INTAKE_SPARK_MAX_ENCODER = new Encoder(Measurements.K_INDEXER_INTAKE_SPARK_MAX_ENCODER_CHANNELA, Measurements.K_INDEXER_INTAKE_SPARK_MAX_ENCODER_CHANNELB);
        CANNON_SPARK_MAX_ENCODER = new Encoder(Measurements.K_INDEXER_CANNON_SPARK_MAX_ENCODER_CHANNELA, Measurements.K_INDEXER_CANNON_SPARK_MAX_ENCODER_CHANNELB);
        ENCODERS = new Encoder[]{INTAKE_SPARK_MAX_ENCODER, CANNON_SPARK_MAX_ENCODER};

        INTAKE_SPARK_MAX_PID_CONTROLLER = INTAKE_SPARK_MAX.getPIDController();
        CANNON_SPARK_MAX_PID_CONTROLLER = CANNON_SPARK_MAX.getPIDController();

        configPID();
        config(INTAKE_SPARK_MAX);
        config(CANNON_SPARK_MAX);

        INTAKE_BREAKBEAM_INPUT = new DigitalInput(Constants.Ports.INDEXER_INTAKE_BREAKBEAM_INPUT_ID);
        CANNON_BREAKBEAM_INPUT = new DigitalInput(Constants.Ports.INDEXER_CANNON_BREAKBEAM_INPUT_ID);
    }
    
    // ---------------------------------------------------------------[Methods]--------------------------------------------------------------- //
    @Override
    public synchronized void periodic() {
        Constants.Objects.ODOMETRY_LOCKER.lock();
        HasNote = !(INTAKE_BREAKBEAM_INPUT.get()|| CANNON_BREAKBEAM_INPUT.get());
        Constants.Objects.ODOMETRY_LOCKER.lock();
    }

    public synchronized static void configPID() {
        INTAKE_SPARK_MAX_PID_CONTROLLER.setFeedbackDevice((MotorFeedbackSensor) INTAKE_SPARK_MAX_ENCODER);
        INTAKE_SPARK_MAX_PID_CONTROLLER.setP(Measurements.INDEXER_INTAKE_SPARK_MAX_P);
        INTAKE_SPARK_MAX_PID_CONTROLLER.setI(Measurements.INDEXER_INTAKE_SPARK_MAX_I);
        INTAKE_SPARK_MAX_PID_CONTROLLER.setD(Measurements.INDEXER_INTAKE_SPARK_MAX_D);

        CANNON_SPARK_MAX_PID_CONTROLLER.setFeedbackDevice((MotorFeedbackSensor) CANNON_SPARK_MAX_ENCODER);
        CANNON_SPARK_MAX_PID_CONTROLLER.setP(Measurements.INDEXER_CANNON_SPARK_MAX_P);
        CANNON_SPARK_MAX_PID_CONTROLLER.setI(Measurements.INDEXER_CANNON_SPARK_MAX_I);
        CANNON_SPARK_MAX_PID_CONTROLLER.setD(Measurements.INDEXER_CANNON_SPARK_MAX_D);
    }

    public synchronized static void config(CANSparkMax motor){
        motor.setIdleMode(IdleMode.kCoast);
        motor.restoreFactoryDefaults();
        motor.clearFaults();
    
        motor.setSmartCurrentLimit(Measurements.K_CURRENT_LIMIT);
    }
    
    /**
     * Closes this instance and all held resources immediately 
     */
    public synchronized void close() {
        INTAKE_SPARK_MAX.close();
        CANNON_SPARK_MAX.close();
    }
    
    // --------------------------------------------------------------[Internal]--------------------------------------------------------------- //
    /**
     * Describes the relationship of a relevant game piece to the position its going to be sent to.
     */
    public enum Direction { //INTAKE    INDEXER    CANNON
        FORWARD_INDEXER,    //   O--------->                (called by INTAKE system to complete a pickup)
        FORWARD_CANNON,     //             O--------->      (called by CANNON system in order to put a Note in firing position)
        BACKWARD_INDEXER,   //   <---------O                (called manually (e.g. Note jammed in singulator))
    }

    // --------------------------------------------------------------[Mutators]--------------------------------------------------------------- //
    /**
     * Indexer moves a game piece a given direction through the indexer subsystem given a direction, e.g. BACKWARD_INDEXER would mean pulling 
     * a held game piece from the indexing chamber back out to the intake.
     * @param Demand Directional Demand, which way for a game piece to be moved.
     */
    public synchronized void set(final Direction Demand) {
        switch(Demand) {
            case FORWARD_INDEXER:
//              MOTORS[] spin at Measurements.INDEXER_MOTOR_SPEED for Measurements.INDEXER_MOTOR_DURATION;
                break;
            case FORWARD_CANNON:
                if (getHoldingNote()) {
//                  MOTORS[] spin at Measurements.INDEXER_MOTOR_SPEED for Measurements.INDEXER_MOTOR_DURATION;
//                  Tell CANNON to continueWithFireSequence;
                } else {
//                  Tell CANNON to abortFireSequence;
                }
                break;
            case BACKWARD_INDEXER:
//              MOTORS[] spin at -(Measurements.INDEXER_MOTOR_SPEED) for Measurements.INDEXER_MOTOR_DURATION;
//              Tell INTAKE to do intake sequence backwards (negative speed, same time, etc etc);
                break;
            default:
//              Stop everything;
                break;
            }
        }

    // --------------------------------------------------------------[Accessors]-------------------------------------------------------------- //
    /**
     * Provides a boolean representation of if this indexer is currently holding a not or not.
     * @return Boolean of if a note is being held
     */
    public Boolean getHoldingNote() {
        return HasNote;
    }
    
    /**
     * Retrieves the existing instance of this static utility class
     * @return Utility class's instance
     */
    public static synchronized IndexerSubsystem getInstance() {
        if (java.util.Objects.isNull(Instance))
            Instance = new IndexerSubsystem();
        return Instance;
    }
}