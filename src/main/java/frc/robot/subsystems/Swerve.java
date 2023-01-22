package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    //public SwerveDriveOdometry swerveOdometry;
    public SwerveDrivePoseEstimator poseEstimator;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;

    public Field2d field = new Field2d();

    private PhotonCamera camera;
    private PhotonPipelineResult result;
    private AprilTagFieldLayout fieldLayout;

    public Swerve() {
        gyro = new Pigeon2(Constants.Swerve.pigeonID);
        gyro.configFactoryDefault();
        zeroGyro();

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        /* By pausing init for a second before setting module offsets, we avoid a bug with inverting motors.
         * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
         */
        Timer.delay(1.0);
        resetModulesToAbsolute();

        //swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getModulePositions());
        poseEstimator = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics, getYaw(), getModulePositions(), new Pose2d(),
                                new MatBuilder<N3, N1>(Nat.N3(), Nat.N1()).fill(0.1, 0.1, 0.1), // State measurement
                                                                                                // standard deviations.
                                                                                                // X, Y, theta.
                                new MatBuilder<N3, N1>(Nat.N3(), Nat.N1()).fill(1.25, 1.25, 1.25)); // Vision measurement
                                                                                                 // standard deviations.
                                                                                                 // X, Y, theta.);
        
        try {
            fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
        } catch (Exception e) {
            System.out.print("Failed to initialize apriltag layout");
        }

        SmartDashboard.putData("Field", field);
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getYaw()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }    

    public Pose2d getPose() {
        //return swerveOdometry.getPoseMeters();
        return poseEstimator.getEstimatedPosition();
    }

    public void resetOdometry(Pose2d pose) {
        //swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
        poseEstimator.resetPosition(getYaw(), getModulePositions(), pose);
    }

    /** Updates the pose estimator from a (presumably vision) measurement
     * Input is in the form of a list of pose2ds and a latency measurement
     */
    public void updateOdometry(Pair<List<Pose2d>, Double> data){
        System.out.println(data.getFirst());

        if (data != null) {
        field.getObject("Latest Vision Pose").setPoses(data.getFirst());
        SmartDashboard.putNumber("Latency", data.getSecond());
        for (Pose2d pose : data.getFirst()){
            // Data is in milliseconds, need to convert to seconds
            poseEstimator.addVisionMeasurement(pose, Timer.getFPGATimestamp() - (data.getSecond() / 1000));
            zeroGyro(pose.getRotation().getDegrees());
        }
        }
    }

    /**Processes the vision result.
   * 
   * @return a pair of the list of poses from each target, and the latency of the result
   */
  public Pair<List<Pose2d>, Double> getEstimatedPose(){
    // Only do work if we actually have targets, if we don't return null
    if (result.hasTargets()){
      // List that we're going to return later
      List<Pose2d> poses = new ArrayList<Pose2d>();
      // Loop through all the targets
      for (PhotonTrackedTarget target : result.getTargets()){
        // Use a switch statement to lookup the pose of the marker
        // Later will switch this to use wpilibs json file to lookup pose of marker
        Pose3d targetPose3d = new Pose3d();
        if (fieldLayout != null && fieldLayout.getTagPose(target.getFiducialId()).isPresent()) {
            targetPose3d = fieldLayout.getTagPose(target.getFiducialId()).get();
        } else if (fieldLayout == null) {
            System.out.println("No field layout");
            continue;
        } else {
            System.out.println("No tag found with that ID");
            continue;
        }
        // Reject targets with a high ambiguity. Threshold should be tuned
        if (target.getPoseAmbiguity() < 0.1) {
          // Calculate and add the pose to our list of poses
          // May need to invert the camera to robot transform?
          poses.add(getFieldToRobot(targetPose3d, Constants.CAMERA_TO_ROBOT, target.getBestCameraToTarget()).toPose2d());
        }
        // Return the list of poses and the latency
        return new Pair<>(poses, result.getLatencyMillis());
      }
    }
    // Returns null if no targets are found
    return null;
  }

    /**
     * Estimates the pose of the robot in the field coordinate system, given the id of the fiducial, the robot relative to the
     * camera, and the target relative to the camera.
     * Stolen from mdurrani834
     * @param tagPose Pose3d the field relative pose of the target
     * @param robotToCamera Transform3d of the robot relative to the camera. Origin of the robot is defined as the center.
     * @param cameraToTarget Transform3d of the target relative to the camera, returned by PhotonVision
     * @return Pose Robot position relative to the field.
     */
    private Pose3d getFieldToRobot(Pose3d tagPose, Transform3d robotToCamera, Transform3d cameraToTarget) {
        return tagPose.plus(cameraToTarget.inverse()).plus(robotToCamera.inverse()); 
    }   

    public double getCameraResultLatency(){
        return result.getLatencyMillis();
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    /** Resets the gyro to a heading of 0 */
    public void zeroGyro(){
        zeroGyro(0);
    }

    /** Resets the gyro to a given heading */
    public void zeroGyro(double angle){
        gyro.setYaw(angle);
    }

    public Rotation2d getYaw() {
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getYaw()) : Rotation2d.fromDegrees(gyro.getYaw());
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    @Override
    public void periodic(){
        //swerveOdometry.update(getYaw(), getModulePositions());
        poseEstimator.update(getYaw(), getModulePositions());  
        
        result = camera.getLatestResult();
        
        if (DriverStation.isDisabled()){
            resetModulesToAbsolute();
        }
        
        field.setRobotPose(getPose());
        
        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }
        SmartDashboard.putNumber("Heading", getYaw().getDegrees());
    }
}