package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drive extends SubsystemBase{

    private AHRS gyro;

    private WPI_TalonSRX fl;
    private WPI_TalonSRX fr;
    private WPI_VictorSPX bl;
    private WPI_VictorSPX br;

    private PIDController rightPID;
    private PIDController leftPID;

    private DifferentialDriveKinematics m_Kinematics;
    private DifferentialDriveOdometry m_Odometry;
    private SimpleMotorFeedforward m_feedForward;
    private final Field2d field2d = new Field2d();

    public Drive(){
        gyro = new AHRS(SPI.Port.kMXP);
        zeroGyro();

        fl = new WPI_TalonSRX(Constants.DriveConstants.frontLeftCAN);
        fr = new WPI_TalonSRX(Constants.DriveConstants.frontRightCAN);
        bl = new WPI_VictorSPX(Constants.DriveConstants.backLeftCAN);
        br = new WPI_VictorSPX(Constants.DriveConstants.backRightCAN);

        bl.follow(fl);
        br.follow(fr);

        fr.setInverted(false);
        fl.setInverted(true);   
        bl.setInverted(true);

        fl.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        fr.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        
        fl.setSelectedSensorPosition(0);
        fr.setSelectedSensorPosition(0);

        //leftPID = new PIDController(Constants.DriveConstants.kP, Constants.DriveConstants.kI, Constants.DriveConstants.kD);
        //rightPID = new PIDController(Constants.DriveConstants.kP, Constants.DriveConstants.kI, Constants.DriveConstants.kD);
 
        m_Kinematics = new DifferentialDriveKinematics(Constants.DriveConstants.kTrackWidth);
        //m_feedForward = new SimpleMotorFeedforward(Constants.DriveConstants.kS, Constants.DriveConstants.kV);
        m_Odometry = new DifferentialDriveOdometry(gyro.getRotation2d());

        SmartDashboard.putData(field2d);
    }

    public void drive(double translation, double rotation){
        var wheelSpeeds = m_Kinematics.toWheelSpeeds(new ChassisSpeeds(translation, 0.0, rotation));
        setSpeeds(wheelSpeeds);
    }

    public void setSpeeds(DifferentialDriveWheelSpeeds speeds){
        /*
        final double leftFeedforward = m_feedForward.calculate(speeds.leftMetersPerSecond);
        1final double rightFeedforward = m_feedForward.calculate(speeds.rightMetersPerSecond);

        final double leftOutput =
            leftPID.calculate(fl.getSelectedSensorVelocity()*Constants.DriveConstants.EncoderConversionFactor, speeds.leftMetersPerSecond);
        final double rightOutput =
            rightPID.calculate(fr.getSelectedSensorVelocity()*Constants.DriveConstants.EncoderConversionFactor, speeds.rightMetersPerSecond);

        fl.setVoltage(leftOutput + leftFeedforward);
        fr.setVoltage(rightOutput + rightFeedforward);
        */

        fl.set(speeds.leftMetersPerSecond/Constants.DriveConstants.maxSpeed);
        fr.set(speeds.rightMetersPerSecond/Constants.DriveConstants.maxSpeed);
        updateOdometry();
    }

    public void updateOdometry(){
        m_Odometry.update(
            gyro.getRotation2d(), fl.getSelectedSensorPosition()/Constants.DriveConstants.EncoderConversionFactor, fr.getSelectedSensorPosition()/Constants.DriveConstants.EncoderConversionFactor);
        field2d.setRobotPose(m_Odometry.getPoseMeters());
        SmartDashboard.putData(field2d);
    }

    public void resetOdometry(Pose2d pose2d) {
        m_Odometry.resetPosition(pose2d, Rotation2d.fromDegrees(gyro.getAngle()));
    }

    public void zeroGyro(){
        gyro.reset();
    }

}
