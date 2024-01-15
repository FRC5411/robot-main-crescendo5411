package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase
{
    private InfraredReceiver receiverOne;
    private InfraredReceiver receiverTwo;
    public bool hasNote; //hasNote is THE bool value that the indexer keeps track of; everything else is secondary to making this accurate 100% of the time

    public Indexer()
    {
        receiverOne = new InfraredReceiver(port: 0); //receivers return a 0 if the robot has a Note and a 1 if not
        receiverTwo = new InfraredReceiver(port: 1); //I think 2 IR beams will be better than 1; no empirical evidence for now, I will be getting that and reexamining
    }
    
    @Override
    public void periodic()
    {
        hasNote = !(receiverOne.status || receiverTwo.status);
        //hasNote will be true if the singulator is holding a Note
        //for now I am using the OR operator for the two beam sensors, but that might turn into an AND depending on what is more reliable
    }
}