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
    
    private static final Encoder INTAKE_SPARK_MAX_ENCODER;
    private static final Encoder CANNON_SPARK_MAX_ENCODER;

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

        INTAKE_SPARK_MAX = new CANSparkMax(Constants.INXR.INTAKE_SPARK_MAX_ID, MotorType.kBrushless);
        CANNON_SPARK_MAX = new CANSparkMax(Constants.INXR.CANNON_SPARK_MAX_ID, MotorType.kBrushless);

        INTAKE_SPARK_MAX_ENCODER = new Encoder(Constants.INXR.K_INTAKE_SPARK_MAX_ENCODER_CHANNELA, Constants.INXR.K_INTAKE_SPARK_MAX_ENCODER_CHANNELB);
        CANNON_SPARK_MAX_ENCODER = new Encoder(Constants.INXR.K_CANNON_SPARK_MAX_ENCODER_CHANNELA, Constants.INXR.K_CANNON_SPARK_MAX_ENCODER_CHANNELB);

        INTAKE_BREAKBEAM_INPUT = new DigitalInput(Constants.INXR.INDEXER_INTAKE_BREAKBEAM_INPUT_ID);
        CANNON_BREAKBEAM_INPUT = new DigitalInput(Constants.INXR.INDEXER_CANNON_BREAKBEAM_INPUT_ID);
    }
    
    // ---------------------------------------------------------------[Methods]--------------------------------------------------------------- //
    @Override
    public synchronized void periodic() {
        Constants.Objects.ODOMETRY_LOCKER.lock();
        HasNote = setHasNote();
        Constants.Objects.ODOMETRY_LOCKER.lock();
    }

    public synchronized static void config(CANSparkMax motor){
        motor.setIdleMode(IdleMode.kBrake);
        motor.restoreFactoryDefaults();
        motor.clearFaults();
    
        motor.setSmartCurrentLimit(Constants.INXR.K_CURRENT_LIMIT);
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
                INTAKE_SPARK_MAX.set(Constants.INXR.INTAKE_SPARK_MAX_SPEED);
                CANNON_SPARK_MAX.set(Constants.INXR.CANNON_SPARK_MAX_SPEED);
//              Wait for Constants.INXR.SPARK_MAX_DURATION;
                INTAKE_SPARK_MAX.set(0);
                CANNON_SPARK_MAX.set(0);
                break;
            case FORWARD_CANNON:
                if (getHoldingNote()) {
                    INTAKE_SPARK_MAX.set(Constants.INXR.INTAKE_SPARK_MAX_SPEED);
                    CANNON_SPARK_MAX.set(Constants.INXR.CANNON_SPARK_MAX_SPEED);
//                  Wait for Constants.INXR.SPARK_MAX_DURATION;
                    INTAKE_SPARK_MAX.set(0);
                    CANNON_SPARK_MAX.set(0);
//                  Tell CANNON to continueWithFireSequence;
                } else {
//                  Tell CANNON to abortFireSequence;
                }
                break;
            case BACKWARD_INDEXER:
                INTAKE_SPARK_MAX.set(-Constants.INXR.INTAKE_SPARK_MAX_SPEED)
                CANNON_SPARK_MAX.set(-Constants.INXR.CANNON_SPARK_MAX_SPEED);
//              Wait for Constants.INXR.SPARK_MAX_DURATION;
                INTAKE_SPARK_MAX.set(0);
                CANNON_SPARK_MAX.set(0);
//              Tell INTAKE to do intake sequence backwards (negative speed, same time, etc etc);
                break;
            default:
                INTAKE_SPARK_MAX.set(0);
                CANNON_SPARK_MAX.set(0);
                break;
        }
    }

    /**
     * Sets the value of HasNote to either true or false.
     * True: both Beam Break sensors read a 0 (beam disconnected)
     * False: either Beam Break reads a 1 (beam connects)
     * 
     * @return new Boolean value of HasNote
     */
    public synchronized Boolean setHasNote()
    {
        return !(INTAKE_BREAKBEAM_INPUT.get() || CANNON_BREAKBEAM_INPUT.get());
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