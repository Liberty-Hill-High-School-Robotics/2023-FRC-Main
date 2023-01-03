// RobotBuilder Version: 4.0
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


import frc.robot.commands.*;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import com.revrobotics.CANSparkMax; 
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS


/**
 *
 */
public class Drive extends SubsystemBase {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
private CANSparkMax cANSparkMAXRF;
private CANSparkMax cANSparkMAXLF;
private DifferentialDrive driveMain;
private CANSparkMax cANSparkMAXRB;
private CANSparkMax cANSparkMAXLB;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    
    /**
    *
    */
    public Drive() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
cANSparkMAXRF = new CANSparkMax(3, MotorType.kBrushless);
 
 

cANSparkMAXLF = new CANSparkMax(2, MotorType.kBrushless);
 
 

driveMain = new DifferentialDrive(cANSparkMAXLF, cANSparkMAXRF);
 addChild("DriveMain",driveMain);
 driveMain.setSafetyEnabled(true);
driveMain.setExpiration(0.1);
driveMain.setMaxOutput(1.0);


cANSparkMAXRB = new CANSparkMax(4, MotorType.kBrushless);
 
 

cANSparkMAXLB = new CANSparkMax(1, MotorType.kBrushless);
 
 


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS

    //sets the two follow motors to follow the lead motors
    cANSparkMAXRB.follow(cANSparkMAXRF);
    cANSparkMAXLB.follow(cANSparkMAXLF);

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.


    public void driveStop() {
        cANSparkMAXRF.stopMotor();
        cANSparkMAXLF.stopMotor();
    }


    //lefty = power    rightx = rotation
    public void driveArcade(double leftY, double rightX) {
        driveMain.arcadeDrive(leftY, rightX);
    }
}

