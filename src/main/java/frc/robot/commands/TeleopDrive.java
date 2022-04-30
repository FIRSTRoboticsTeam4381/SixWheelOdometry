package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.Controls;
import frc.robot.subsystems.Drive;

public class TeleopDrive extends CommandBase{
    
    private Drive d_Drive;

    private Joystick controller;
    private int translationAxis;
    private int rotationAxis;

    private final Field2d m_field = new Field2d();

    private double translation;
    private double rotation;

    private final SlewRateLimiter m_speedLimiter = new SlewRateLimiter(Constants.Controls.slewRateLimit);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(Constants.Controls.slewRateLimit);


    public TeleopDrive(Drive d_Drive, Joystick controller, int translationAxis, int rotationAxis){
        this.d_Drive = d_Drive;
        this.controller = controller;
        this.translationAxis = translationAxis;
        this.rotationAxis = rotationAxis;

        SmartDashboard.putData("Field", m_field);
    }

    @Override
    public void execute(){
        double yAxis = -controller.getRawAxis(translationAxis);
        double rAxis = -controller.getRawAxis(rotationAxis);

        translation = m_speedLimiter.calculate(yAxis) * Constants.Drive.maxSpeed;
        rotation = m_rotLimiter.calculate(rAxis) * Constants.Drive.maxAngularVelocity;

        d_Drive.drive(translation, rotation);
    }


}
