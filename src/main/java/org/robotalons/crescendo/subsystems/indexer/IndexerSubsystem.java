// ----------------------------------------------------------------[Package]---------------------------------------------------------------- //
package org.robotalons.crescendo.subsystems.indexer;

// ---------------------------------------------------------------[Libraries]--------------------------------------------------------------- //
import java.io.Closeable;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.robotalons.crescendo.subsystems.indexer.SubsysConstants.INDEXER;

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
    private final static CANSparkMax TO_CANNON_MOTOR;

    private final static DigitalInput INTAKE_BREAKBEAM_INPUT;
    private final static DigitalInput CANNON_BREAKBEAM_INPUT;

    // ---------------------------------------------------------------[Fields]---------------------------------------------------------------- //
    private static IndexerSubsystem Instance;

    private static Boolean IntakeBeamSeesNote;
    private static Boolean CannonBeamSeesNote;

    // ------------------------------------------------------------[Constructors]------------------------------------------------------------- //
    /**
     * Indexer Subsystem Constructor
     */
    public IndexerSubsystem() {} static {
        Instance = new IndexerSubsystem();

        TO_CANNON_MOTOR = new CANSparkMax(INDEXER.TO_CANNON_MOTOR_ID, MotorType.kBrushless);

        INTAKE_BREAKBEAM_INPUT = new DigitalInput(INDEXER.INTAKE_BREAKBEAM_INPUT_ID);
        CANNON_BREAKBEAM_INPUT = new DigitalInput(INDEXER.CANNON_BREAKBEAM_INPUT_ID);

        IntakeBeamSeesNote = (false);
        CannonBeamSeesNote = (false);
    }
    
    // ---------------------------------------------------------------[Methods]--------------------------------------------------------------- //
    @Override
    public synchronized void periodic() {
        INDEXER.ODOMETRY_LOCKER.lock();
        
        boolean newIntakeBeamSeesNote = !(INTAKE_BREAKBEAM_INPUT.get());
        boolean newCannonBeamSeesNote = !(CANNON_BREAKBEAM_INPUT.get());
        
        if (IntakeBeamSeesNote != newIntakeBeamSeesNote) {
            IntakeBeamSeesNote = newIntakeBeamSeesNote;
            if (IntakeBeamSeesNote) {
                TO_CANNON_MOTOR.set(INDEXER.TO_CANNON_MOTOR_SPEED);
                //INDEX_LED.set(off);
            } else {
                TO_CANNON_MOTOR.set(INDEXER.TO_CANNON_MOTOR_SPEED);
                //INDEX_LED.set(off);
                //IntakeSubsystem.INTAKE_MOTOR.set(0);
            }
        }

        if (CannonBeamSeesNote != newCannonBeamSeesNote) {
            CannonBeamSeesNote = newCannonBeamSeesNote;
            if (CannonBeamSeesNote) {
                TO_CANNON_MOTOR.set(0);
                //INDEX_LED.set(green, flashing);
                //IntakeSubsystem.INTAKE_MOTOR.set(0);
            } else {
                TO_CANNON_MOTOR.set(0);
                //INDEX_LED.set(off);
            }
        }

        INDEXER.ODOMETRY_LOCKER.lock();
    }

    public synchronized static void config(CANSparkMax motor){
        motor.setIdleMode(IdleMode.kBrake);
        motor.restoreFactoryDefaults();
        motor.clearFaults();
    
        motor.setSmartCurrentLimit(INDEXER.K_CURRENT_LIMIT);
    }
    
    /**
     * Closes this instance and all held resources immediately 
     */
    public synchronized void close() {
        TO_CANNON_MOTOR.close();
    }
    
    // --------------------------------------------------------------[Internal]--------------------------------------------------------------- //
    

    // --------------------------------------------------------------[Mutators]--------------------------------------------------------------- //
    /**
     * Called by the Cannon subsystem before firing a Note.
     * Serves the dual purpose of checking for the presence of a Note and passing any Note up to the Cannon.
     * 
     * @return Boolean value of whether it is safe for Cannon to proceed with firing sequence (Note is present and clear of TO_CANNON_MOTOR)
     */
    public static synchronized Boolean PassToCannon() {
        boolean safeToFire = false;
        if (CannonBeamSeesNote) {
            TO_CANNON_MOTOR.set(INDEXER.TO_CANNON_MOTOR_SPEED);
            while (CannonBeamSeesNote) {} //placeholder while I learn of a better way to delay
            safeToFire = true;
        }
        return safeToFire;
    }

    // --------------------------------------------------------------[Accessors]-------------------------------------------------------------- //
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