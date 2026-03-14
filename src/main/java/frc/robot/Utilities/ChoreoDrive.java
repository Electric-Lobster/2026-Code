// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Utilities;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import choreo.trajectory.DifferentialSample;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.controller.LTVUnicycleController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Drivetrain for choreo based robots */
public class ChoreoDrive {

    PhotonVisionProccesing photon;

    //Enums
    enum EncoderReturn{
        kDistance,
        kSpeed,
        kLeftDistance,
        kLeftSpeed,
        kRightDistance,
        kRightSpeed
    }


    //Constants
    class StoredConstants {
        public StoredConstants() {

        }
        
        public int stallCurrent = 40;
        public int freeCurrent = 50;
        public int stallSpeed = 100;

        public double posConversion = (1.0 / 8.45) * 0.1524 * Math.PI;
        public double velConversion = ( (1.0 / 8.45) * 0.1524 * Math.PI) * (1.0/60.0);

        public boolean inverted = false;

        public IdleMode idlemode = IdleMode.kBrake;

        public double trackWidthMeters = 0.60325;

        public double[] velPID = {0.05, 0, 0};
    }

    public StoredConstants CONST = new StoredConstants();


    //Motors
    private SparkMax fLdrive;
    private SparkMax fRDrive;
    private SparkMax bLDrive;
    private SparkMax bRDrive;


    //Encoders
    private RelativeEncoder fLEncoder;
    private RelativeEncoder fREncoder;
    private RelativeEncoder bLEncoder;
    private RelativeEncoder bREncoder;


    //PIDS
    PIDController lVelPID;
    PIDController rVelPID;


    //GYRO
    private ADIS16470_IMU Gyro = new ADIS16470_IMU();


    //Choreo Specific
    LTVUnicycleController trajectoryController = new LTVUnicycleController(0.01);
    DifferentialDriveKinematics kinematics;
    DifferentialDriveOdometry odometry;
    Pose2d initialPose = new Pose2d();

    //Field
    private Field2d field = new Field2d();


    public ChoreoDrive(String camName, Transform3d camToCenter) {
        SmartDashboard.putData(field);

        photon = new PhotonVisionProccesing(camName, camToCenter);
    }

    public ChoreoDrive(double posConversion, double velConversion, String camName, Transform3d camToCenter) {
        CONST.posConversion = posConversion;
        CONST.velConversion = velConversion;

        SmartDashboard.putData(field);

        photon = new PhotonVisionProccesing(camName, camToCenter);
    }

    public ChoreoDrive(double posConversion, Double velConversion, int stallCurrent, int freeCurrent, String camName, Transform3d camToCenter) {
        CONST.posConversion = posConversion;
        CONST.velConversion = velConversion;
        
        CONST.stallCurrent = stallCurrent;
        CONST.freeCurrent = freeCurrent;

        SmartDashboard.putData(field);

        photon = new PhotonVisionProccesing(camName, camToCenter);
    }


    public void ConfigureMotors(SparkMax FL, SparkMax FR, SparkMax BL, SparkMax BR) {
        this.fLdrive = FL;
        this.fRDrive = FR;
        this.bLDrive = BL;
        this.bRDrive = BR;

        SparkMaxConfig FLConfig = new SparkMaxConfig();
        SparkMaxConfig FRConfig = new SparkMaxConfig();
        SparkMaxConfig BLConfig = new SparkMaxConfig();
        SparkMaxConfig BRConfig = new SparkMaxConfig();

        FLConfig.encoder.positionConversionFactor(CONST.posConversion).velocityConversionFactor(CONST.velConversion);
        FLConfig.smartCurrentLimit(CONST.stallCurrent, CONST.freeCurrent, 100).inverted(CONST.inverted).idleMode(CONST.idlemode);

        FRConfig.encoder.positionConversionFactor(CONST.posConversion).velocityConversionFactor(CONST.velConversion);
        FRConfig.smartCurrentLimit(CONST.stallCurrent, CONST.freeCurrent, 100).inverted(!CONST.inverted).idleMode(CONST.idlemode);

        BLConfig.encoder.positionConversionFactor(CONST.posConversion).velocityConversionFactor(CONST.velConversion);
        BLConfig.smartCurrentLimit(CONST.stallCurrent, CONST.freeCurrent, 100).inverted(CONST.inverted).idleMode(CONST.idlemode).follow(fLdrive);

        BRConfig.encoder.positionConversionFactor(CONST.posConversion).velocityConversionFactor(CONST.velConversion);
        BRConfig.smartCurrentLimit(CONST.stallCurrent, CONST.freeCurrent, 100).inverted(!CONST.inverted).idleMode(CONST.idlemode).follow(fRDrive);

        fLdrive.configure(FLConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        fRDrive.configure(FRConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        bLDrive.configure(BLConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        bRDrive.configure(BRConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        fLEncoder = fLdrive.getEncoder();
        fREncoder = fRDrive.getEncoder();
        bLEncoder = bLDrive.getEncoder();
        bREncoder = bRDrive.getEncoder();

        lVelPID = new PIDController(CONST.velPID[0], CONST.velPID[0], CONST.velPID[0]);
        rVelPID = new PIDController(CONST.velPID[0], CONST.velPID[0], CONST.velPID[0]);

        kinematics = new DifferentialDriveKinematics(CONST.trackWidthMeters);
        Gyro.calibrate();
        resetEncoder();
        odometry = new DifferentialDriveOdometry(new Rotation2d(Math.toRadians(getYaw())), getEncoder(EncoderReturn.kLeftDistance), getEncoder(EncoderReturn.kRightDistance));
    }


    public double getEncoder(EncoderReturn returnType) {
        switch (returnType) {
            case kDistance:
                return ( fLEncoder.getPosition() + fREncoder.getPosition() + bLEncoder.getPosition() + bREncoder.getPosition() ) / 4.0;
            
            case kSpeed:
                return ( fLEncoder.getVelocity() + fREncoder.getVelocity() + bLEncoder.getVelocity() + bREncoder.getVelocity() ) / 4.0;

            case kLeftDistance:
                return ( fLEncoder.getPosition() + bLEncoder.getPosition() ) / 2.0;

            case kLeftSpeed:
                return ( fLEncoder.getVelocity() + bLEncoder.getVelocity() ) / 2.0;

            case kRightDistance:
                return ( fREncoder.getPosition() + bREncoder.getPosition() ) / 2.0;

            case kRightSpeed:
                return ( fREncoder.getVelocity() + bREncoder.getVelocity() ) / 2.0;
            
            default:
                return 0;
        }
    }


    public void resetEncoder() {
        fLEncoder.setPosition(0);
        fREncoder.setPosition(0);
        bLEncoder.setPosition(0);
        bREncoder.setPosition(0);
    }


    public double getYaw() {
        return Gyro.getAngle(Gyro.getYawAxis());
    }


    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }


    public void resetPose(Pose2d newPose) {
        initialPose = newPose;
        odometry.resetPose(newPose);
        resetEncoder();
    }


    /**
    * follows a trajectory for autonomous programs
    * @param sample the current sample of the trajectory
    */
    public void followTrajectory(DifferentialSample sample) {

        //gets robot pose
        Pose2d pose = getPose();

        // velocity from sample
        ChassisSpeeds ff = sample.getChassisSpeeds();

        //generate next robot speed
        ChassisSpeeds speeds = trajectoryController.calculate(pose, sample.getPose(), ff.vxMetersPerSecond, ff.omegaRadiansPerSecond);

        DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);
        velocityDrive(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond);
    }


    /**
    * Drives the robots using speed for both sides of the drivetrain
    * @param leftSpeed left wheel velocity
    * @param rightSpeed right wheel velocity
    */
    private void velocityDrive(double leftSpeed, double rightSpeed) {
        lVelPID.setSetpoint(leftSpeed);
        rVelPID.setSetpoint(rightSpeed);

        tankDrive(lVelPID.calculate(getEncoder(EncoderReturn.kLeftSpeed)), rVelPID.calculate(getEncoder(EncoderReturn.kRightSpeed)));
    }


    /**
    * drives the robot in tank style controls
    * @param left throttle
    * @param right throttle
    */
    public void tankDrive(double left, double right) {

        fLdrive.set(left);
        fRDrive.set(right);
    }


    public void periodic() {
        if (photon.foundTag() && (Math.abs(fLdrive.get()) < 0.1) && (Math.abs(fRDrive.get()) <0.1)) {
            resetPose(photon.GetBestPose(initialPose));
        }

        odometry.update(new Rotation2d(Math.toRadians(getYaw())), getEncoder(EncoderReturn.kLeftDistance), getEncoder(EncoderReturn.kRightDistance));
        
        field.setRobotPose(getPose());

    }

}
