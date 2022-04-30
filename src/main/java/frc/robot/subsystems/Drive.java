package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drive extends SubsystemBase{

    private AHRS gyro;

    private CANSparkMax fl;
    private CANSparkMax fr;
    private CANSparkMax bl;
    private CANSparkMax br;

    private PIDController rightPID;
    private PIDController leftPID;

    private RelativeEncoder rightEnc;
    private RelativeEncoder leftEnc;

    private DifferentialDriveKinematics m_Kinematics;
    private DifferentialDriveOdometry m_Odometry;
    private SimpleMotorFeedforward m_feedForward;

    public Drive(){
        gyro = new AHRS(SPI.Port.kMXP);
        zeroGyro();

        fl = new CANSparkMax(Constants.Drive.frontLeftCAN, MotorType.kBrushless);
        fr = new CANSparkMax(Constants.Drive.frontRightCAN, MotorType.kBrushless);
        bl = new CANSparkMax(Constants.Drive.backLeftCAN, MotorType.kBrushless);
        br = new CANSparkMax(Constants.Drive.backRightCAN, MotorType.kBrushless);

        bl.follow(fl);
        br.follow(fr);

        fr.setInverted(true);

        leftEnc = fl.getEncoder();
        rightEnc = fr.getEncoder();

        leftEnc.setPositionConversionFactor(Constants.Drive.EncoderConversionFactor);
        rightEnc.setPositionConversionFactor(Constants.Drive.EncoderConversionFactor);
        
        leftEnc.setPosition(0);
        rightEnc.setPosition(0);

        leftPID = new PIDController(Constants.Drive.kP, Constants.Drive.kI, Constants.Drive.kD);
        rightPID = new PIDController(Constants.Drive.kP, Constants.Drive.kI, Constants.Drive.kD);

        m_Kinematics = new DifferentialDriveKinematics(Constants.Drive.kTrackWidth);
        m_feedForward = new SimpleMotorFeedforward(Constants.Drive.kS, Constants.Drive.kV);
        m_Odometry = new DifferentialDriveOdometry(gyro.getRotation2d());
    }

    public void drive(double translation, double rotation){
        var wheelSpeeds = m_Kinematics.toWheelSpeeds(new ChassisSpeeds(translation, 0.0, rotation));
        setSpeeds(wheelSpeeds);
    }

    public void setSpeeds(DifferentialDriveWheelSpeeds speeds){
        final double leftFeedforward = m_feedForward.calculate(speeds.leftMetersPerSecond);
        final double rightFeedforward = m_feedForward.calculate(speeds.rightMetersPerSecond);

        final double leftOutput =
            leftPID.calculate(leftEnc.getVelocity(), speeds.leftMetersPerSecond);
        final double rightOutput =
            rightPID.calculate(rightEnc.getVelocity(), speeds.rightMetersPerSecond);

        fl.setVoltage(leftOutput + leftFeedforward);
        fr.setVoltage(rightOutput + rightFeedforward);
    }

    public void updateOdometry(){
        m_Odometry.update(
            gyro.getRotation2d(), leftEnc.getPosition(), rightEnc.getPosition());
    }

    public void zeroGyro(){
        gyro.reset();
    }

}
