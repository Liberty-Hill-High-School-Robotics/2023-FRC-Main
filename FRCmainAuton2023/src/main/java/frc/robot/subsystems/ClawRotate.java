// RobotBuilder Version: 5.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: Subsystem.

package frc.robot.subsystems;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


/**
 *
 */
public class ClawRotate extends SubsystemBase {

private CANSparkMax rotateClawMotor;

    private SparkMaxLimitSwitch forwardLimit;
    private SparkMaxLimitSwitch reverseLimit;

    private RelativeEncoder encoderCR;

    private AbsoluteEncoder encoderCRAbsolute;
    private double rotatePower = 0.5;

    /**
    *
    */
    public ClawRotate() {
 
rotateClawMotor = new CANSparkMax(5, MotorType.kBrushless);
rotateClawMotor.restoreFactoryDefaults();
rotateClawMotor.setInverted(false);

forwardLimit = rotateClawMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
reverseLimit = rotateClawMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);

forwardLimit.enableLimitSwitch(true);
reverseLimit.enableLimitSwitch(true);

    encoderCR = rotateClawMotor.getEncoder(Type.kHallSensor, 42);
    // encoderCRAbsolute = rotateClawMotor.getAbsoluteEncode();
;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Claw Rotate Position", encoderCR.getPosition());
        SmartDashboard.putNumber("Claw Rotate Velocity", encoderCR.getVelocity());

        SmartDashboard.putBoolean("Forward Limit Switch", forwardLimit.isPressed());
        SmartDashboard.putBoolean("Reverse Limit Switch", reverseLimit.isPressed());
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation
        
    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    
    public void rotateClawStop() {
        rotateClawMotor.stopMotor();
    }
    
    public void rotateClawDown() {
        rotateClawMotor.set(-rotatePower);
    }
    public void rotateClawUp() {
        rotateClawMotor.set(rotatePower);
    }



}

