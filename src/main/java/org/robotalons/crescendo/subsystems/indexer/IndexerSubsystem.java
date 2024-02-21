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
    private static Boolean IntakeBeamIntact;
    private static Boolean CannonBeamIntact;

    private static Boolean passingForwardIndexer;
    private static Boolean passingForwardCannon;
    private static Boolean passingBackwardIntake;

    // ------------------------------------------------------------[Constructors]------------------------------------------------------------- //
    /**
     * Indexer Subsystem Constructor
     */
    public IndexerSubsystem() {} static {
        Instance = new IndexerSubsystem();

        INTAKE_SPARK_MAX = new CANSparkMax(Constants.INDEXER.INTAKE_SPARK_MAX_ID, MotorType.kBrushless);
        CANNON_SPARK_MAX = new CANSparkMax(Constants.INDEXER.CANNON_SPARK_MAX_ID, MotorType.kBrushless);

        INTAKE_SPARK_MAX_ENCODER = new Encoder(Constants.INDEXER.K_INTAKE_SPARK_MAX_ENCODER_CHANNELA, Constants.INDEXER.K_INTAKE_SPARK_MAX_ENCODER_CHANNELB);
        CANNON_SPARK_MAX_ENCODER = new Encoder(Constants.INDEXER.K_CANNON_SPARK_MAX_ENCODER_CHANNELA, Constants.INDEXER.K_CANNON_SPARK_MAX_ENCODER_CHANNELB);

        INTAKE_BREAKBEAM_INPUT = new DigitalInput(Constants.INDEXER.INDEXER_INTAKE_BREAKBEAM_INPUT_ID);
        CANNON_BREAKBEAM_INPUT = new DigitalInput(Constants.INDEXER.INDEXER_CANNON_BREAKBEAM_INPUT_ID);

        HasNote = (false);
        IntakeBeamIntact = (false);
        CannonBeamIntact = (false);

        passingNoteForwardIndexer = (false);
        passingNoteForwardCannon = (false);
        passingNoteBackwardIntake = (false);
    }
    
    // ---------------------------------------------------------------[Methods]--------------------------------------------------------------- //
    @Override
    public synchronized void periodic() {
        Constants.Objects.ODOMETRY_LOCKER.lock();
        
        IntakeBeamIntact = INTAKE_BREAKBEAM_INPUT.get();
        CannonBeamIntact = CANNON_BREAKBEAM_INPUT.get();
        HasNote = setHasNote();

        if (passingNoteForwardIndexer || (passingNoteForwardCannon || passingNoteBackwardIntake))
        {
            if (passingForwardIndexer && HasNote) {
                INTAKE_SPARK_MAX.set(0);
                CANNON_SPARK_MAX.set(0);
                setPassingVars(false);
            } else if (passingForwardCannon && CannonBeamIntact) {
                INTAKE_SPARK_MAX.set(0);
                CANNON_SPARK_MAX.set(0);
                setPassingVars(false);
//              Tell CANNON to continueWithFireSequence;
            } else if (passingBackwardIntake && !IntakeBeamIntact) {
                INTAKE_SPARK_MAX.set(0);
                CANNON_SPARK_MAX.set(0);
                setPassingVars(false);
//              Tell INTAKE to do intake sequence backwards (negative speed, same time, etc etc);
            }
        }

        Constants.Objects.ODOMETRY_LOCKER.lock();
    }

    public synchronized static void config(CANSparkMax motor){
        motor.setIdleMode(IdleMode.kBrake);
        motor.restoreFactoryDefaults();
        motor.clearFaults();
    
        motor.setSmartCurrentLimit(Constants.INDEXER.K_CURRENT_LIMIT);
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
     * Formatted as DIRECTION_DESTINATION
     */
    public enum Direction { //INTAKE    INDEXER    CANNON
        FORWARD_INDEXER,    //   O--------->                (called by INTAKE system to complete a pickup)
        FORWARD_CANNON,     //             O--------->      (called by CANNON system in order to put a Note in firing position)
        BACKWARD_INTAKE,    //   <---------O                (called manually (e.g. Note jammed in singulator))
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
                INTAKE_SPARK_MAX.set(Constants.INDEXER.INTAKE_SPARK_MAX_SPEED);
                CANNON_SPARK_MAX.set(Constants.INDEXER.CANNON_SPARK_MAX_SPEED);
                passingForwardIndexer = (true);
                break;
            case FORWARD_CANNON:
                if (getHoldingNote()) {
                    INTAKE_SPARK_MAX.set(Constants.INDEXER.INTAKE_SPARK_MAX_SPEED);
                    CANNON_SPARK_MAX.set(Constants.INDEXER.CANNON_SPARK_MAX_SPEED);
                    passingForwardCannon = (true);
                } else {
                    INTAKE_SPARK_MAX.set(0);
                    CANNON_SPARK_MAX.set(0);
                    setPassingVars(false);
//                  Tell CANNON to abortFireSequence;
                }
                break;
            case BACKWARD_INTAKE:
                if (getHoldingNote())
                {
                    INTAKE_SPARK_MAX.set(Constants.INDEXER.INTAKE_SPARK_MAX_SPEED);
                    CANNON_SPARK_MAX.set(Constants.INDEXER.CANNON_SPARK_MAX_SPEED);
                    passingBackwardIntake = (true);
                }
                else {
                    INTAKE_SPARK_MAX.set(0);
                    CANNON_SPARK_MAX.set(0);
                    setPassingVars(false);
                }
                break;
            default:
                INTAKE_SPARK_MAX.set(0);
                CANNON_SPARK_MAX.set(0);
                setPassingVars(false);
                break;
        }
    }

    /**
     * Sets the value of HasNote to either true or false. HasNote is only true when a Note is HELD STILL in the Indexer.
     * True: both Beam Break sensors read a 0 (beam disconnected)
     * False: either Beam Break reads a 1 (beam connects)
     * 
     * @return new Boolean value of HasNote
     */
    public synchronized Boolean setHasNote()
    {
        return !(IntakeBeamIntact || CannonBeamIntact);
    }

    public synchronized void setPassingVars(Boolean newPassing)
    {
        passingForwardIndexer = newPassing;
        passingForwardCannon = newPassing;
        passingBackwardIntake = newPassing;
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