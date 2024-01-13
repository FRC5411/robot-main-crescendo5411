// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.lib.motion.kinematics;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N4;
// ------------------------------------------------[Swerve Drivebase Second Order Kinematics]-----------------------------------------------//
/**
 * 
 * @author FRC TEAM 4481
 * 
 */
@Deprecated(forRemoval = true)
public class SwerveDrivebaseSecondOrderKinematics {
  // ---------------------------------------------------------------[Fields]----------------------------------------------------------------//
  private final Translation2d[] ModuleLocations = new Translation2d[4];
  // ------------------------------------------------------------[Constructors]-------------------------------------------------------------//
  /**
   * Swerve Drivebase Second Order Kinematics Constructor.
   * @param FrontLeftModuleLocation  Location of front left swerve module in meters as Translation2d object
   * @param FrontRightModuleLocation Location of front right swerve module in meters as Translation2d object
   * @param RearLeftModuleLocation   Location of back left swerve module in meters as Translation2d object
   * @param RearRightModuleLocation  Location of back right swerve module in meters as Translation2d object
   */
  public SwerveDrivebaseSecondOrderKinematics(
      Translation2d FrontLeftModuleLocation,
      Translation2d FrontRightModuleLocation,
      Translation2d RearLeftModuleLocation,Translation2d RearRightModuleLocation){
    ModuleLocations[0] = FrontLeftModuleLocation;
    ModuleLocations[1] = FrontRightModuleLocation;
    ModuleLocations[2] = RearLeftModuleLocation;
    ModuleLocations[3] = RearRightModuleLocation;
  }
  // ---------------------------------------------------------------[Methods]---------------------------------------------------------------//
  /**
   * Convert chassis speed to states of individual modules using second order kinematics
   *
   * @param Speeds Desired robot translation and rotations
   * @param Heading Heading of the robot, field relative
   * @return {@link SwerveModuleSecondOrderStates} array
   */
  public SwerveModuleSecondOrderStates toSwerveModuleStates(ChassisSpeeds Speeds, Rotation2d Heading){
    Matrix<N3, N1> FirstOrderInput = new Matrix<>(Nat.N3(),Nat.N1());
    Matrix<N2, N3> FirstOrderMatrix = new Matrix<>(Nat.N2(),Nat.N3());
    Matrix<N4, N1> SecondOrderInput = new Matrix<>(Nat.N4(),Nat.N1());
    Matrix<N2, N4> SecondOrderMatrix = new Matrix<>(Nat.N2(),Nat.N4());
    Matrix<N2, N2> RotationMatrix = new Matrix<>(Nat.N2(),Nat.N2());

    FirstOrderInput.set(0,0, Speeds.vxMetersPerSecond);
    FirstOrderInput.set(1,0, Speeds.vyMetersPerSecond);
    FirstOrderInput.set(2,0, Speeds.omegaRadiansPerSecond);

    SecondOrderInput.set(2,0,Math.pow(Speeds.omegaRadiansPerSecond, 2));

    FirstOrderMatrix.set(0,0,1);
    FirstOrderMatrix.set(1,1,1);

    SecondOrderMatrix.set(0,0,1);
    SecondOrderMatrix.set(1,1,1);

    SwerveModuleState[] ModuleStates = new SwerveModuleState[4];
    double[] AzimuthSpeeds = new double[4];
    for (int i = 0; i < 4; i ++){
        Rotation2d moduleAngle = new Rotation2d(Math.atan2(ModuleLocations[i].getY(), ModuleLocations[i].getX())); //Angle that the module location vector makes with respect to the robot
        Rotation2d moduleAngleFieldCentric = moduleAngle.plus(Heading); //Angle that the module location vector makes with respect to the field
        double moduleX = ModuleLocations[i].getNorm() * Math.cos(moduleAngleFieldCentric.getRadians());
        double moduleY = ModuleLocations[i].getNorm() * Math.sin(moduleAngleFieldCentric.getRadians());
        FirstOrderMatrix.set(0,2,-moduleY); //-r_y
        FirstOrderMatrix.set(1,2,moduleX); //r_x

        Matrix<N2, N1> firstOrderOutput = FirstOrderMatrix.times(FirstOrderInput);

        double moduleHeading = Math.atan2(firstOrderOutput.get(1,0), firstOrderOutput.get(0,0));
        double moduleSpeed = Math.sqrt(firstOrderOutput.elementPower(2).elementSum());

        SecondOrderMatrix.set(0,2,-moduleX);
        SecondOrderMatrix.set(0,3,-moduleY);
        SecondOrderMatrix.set(1,2,-moduleY);
        SecondOrderMatrix.set(1,3,moduleX);

        RotationMatrix.set(0,0,Math.cos(moduleHeading));
        RotationMatrix.set(0,1,Math.sin(moduleHeading));
        RotationMatrix.set(1,0,-Math.sin(moduleHeading));
        RotationMatrix.set(1,1,Math.cos(moduleHeading));

        Matrix<N2,N1> secondOrderOutput = RotationMatrix.times(SecondOrderMatrix.times(SecondOrderInput));

        ModuleStates[i] = new SwerveModuleState(moduleSpeed, new Rotation2d(moduleHeading).minus(Heading));
        AzimuthSpeeds[i] = secondOrderOutput.get(1,0) / moduleSpeed - Speeds.omegaRadiansPerSecond;
    }
    return new SwerveModuleSecondOrderStates(ModuleStates, AzimuthSpeeds);
  }
}
