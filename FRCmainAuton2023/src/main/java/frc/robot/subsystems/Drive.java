package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.DriveArcade;

import com.revrobotics.SparkMaxPIDController;

import frc.robot.Robot;
import frc.robot.RobotContainer;



/**
 *
 */

public class Drive extends SubsystemBase {

    private CANSparkMax rightLeader;
    private CANSparkMax leftLeader;
    private DifferentialDrive driveMain;
    private CANSparkMax rightFollow;
    private CANSparkMax leftFollow;

    private RelativeEncoder encoderLeftFollow;
    private RelativeEncoder encoderRightFollow;
    private RelativeEncoder encoderLeftLeader;
    private RelativeEncoder encoderRightLeader;

    private double oldAxisY = 0;
    private double finalAxisY;
    private double tempAxisY;
    private Pigeon2 pigeon2;

    private RelativeEncoder m_encoderLeft;
    private RelativeEncoder m_encoderRight;
    private SparkMaxPIDController m_pidControllerLeft;
    private SparkMaxPIDController m_pidControllerRight;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;

    private double balanceError = 1;
    private double slowPower = 0.1;
    private boolean isBalanced = false;
    private  double setPointLeft;
    private double setPointRight; 
    

    /**
    *
    */
    public Drive() {

        rightLeader = new CANSparkMax(3, MotorType.kBrushless);
        rightLeader.restoreFactoryDefaults();
        rightLeader.setInverted(false);

        leftLeader = new CANSparkMax(2, MotorType.kBrushless);
        leftLeader.restoreFactoryDefaults();
        leftLeader.setInverted(false);

        driveMain = new DifferentialDrive(leftLeader, rightLeader);
        addChild("DriveMain", driveMain);
        driveMain.setSafetyEnabled(true);
        driveMain.setExpiration(0.1);
        driveMain.setMaxOutput(1.0);

        rightFollow = new CANSparkMax(4, MotorType.kBrushless);
        rightFollow.restoreFactoryDefaults();
        // rightFollow.setInverted(false);
        rightFollow.follow(rightLeader, false);

        leftFollow = new CANSparkMax(1, MotorType.kBrushless);
        leftFollow.restoreFactoryDefaults();
        leftFollow.follow(leftLeader, false); // Same direction as leader






        pigeon2 = new Pigeon2(19);


        // sets the two follow motors to follow the lead motors
        final Joystick driverJoystick = new Joystick(0);
        final XboxController operatorController = new XboxController(1);    
       

        
        rightLeader.restoreFactoryDefaults();
        rightFollow.restoreFactoryDefaults();
        leftLeader.restoreFactoryDefaults();
        leftFollow.restoreFactoryDefaults();


        rightFollow.follow(rightLeader);
        leftFollow.follow(leftLeader);


        encoderLeftFollow = leftFollow.getEncoder(Type.kHallSensor, 42);
        encoderLeftFollow.setVelocityConversionFactor(1);

        encoderRightFollow = rightFollow.getEncoder(Type.kHallSensor, 42);
        encoderRightFollow.setVelocityConversionFactor(1);


        encoderLeftLeader = leftLeader.getEncoder(Type.kHallSensor, 42);
        encoderLeftLeader.setVelocityConversionFactor(1);

        encoderRightLeader = rightLeader.getEncoder(Type.kHallSensor, 42);
        encoderRightLeader.setVelocityConversionFactor(1);


        //PID controller stuff

        //create pid
        m_pidControllerLeft = leftLeader.getPIDController();
        m_pidControllerRight = rightLeader.getPIDController();

        //Encoder object created to display position values
        m_encoderLeft = leftLeader.getEncoder();
        m_encoderRight = rightLeader.getEncoder();

        //DO NOT CHANGE (please)
        // PID coefficients
        kP = 0.00006;
        kI = 0.0000006;
        kD = 0; 
        kIz = 0; 
        kFF = 0.000015;  //.00015 default
        kMaxOutput = 1; 
        kMinOutput = -1;
        maxRPM = 3500; //max rpm (goal)

        // set PID coefficients
        m_pidControllerLeft.setP(kP);
        m_pidControllerLeft.setI(kI);
        m_pidControllerLeft.setD(kD);
        m_pidControllerLeft.setIZone(kIz);
        m_pidControllerLeft.setFF(kFF);
        m_pidControllerLeft.setOutputRange(kMinOutput, kMaxOutput);

        m_pidControllerRight.setP(kP);
        m_pidControllerRight.setI(kI);
        m_pidControllerRight.setD(kD);
        m_pidControllerRight.setIZone(kIz);
        m_pidControllerRight.setFF(kFF);
        m_pidControllerRight.setOutputRange(kMinOutput, kMaxOutput);

        // display PID coefficients on SmartDashboard
        SmartDashboard.putNumber("P Gain", kP);
        SmartDashboard.putNumber("I Gain", kI);
        SmartDashboard.putNumber("D Gain", kD);
        SmartDashboard.putNumber("I Zone", kIz);
        SmartDashboard.putNumber("Feed Forward", kFF);
        SmartDashboard.putNumber("Max Output", kMaxOutput);
        SmartDashboard.putNumber("Min Output", kMinOutput);
  }

  

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        //PID stuff
        // read PID coefficients from SmartDashboard
        double p = SmartDashboard.getNumber("P Gain", 0);
        double i = SmartDashboard.getNumber("I Gain", 0);
        double d = SmartDashboard.getNumber("D Gain", 0);
        double iz = SmartDashboard.getNumber("I Zone", 0);
        double ff = SmartDashboard.getNumber("Feed Forward", 0);
        double max = SmartDashboard.getNumber("Max Output", 0);
        double min = SmartDashboard.getNumber("Min Output", 0);

        // if PID coefficients on SmartDashboard have changed, write new values to controller
        if((p != kP)) { m_pidControllerLeft.setP(p); kP = p; }
        if((i != kI)) { m_pidControllerLeft.setI(i); kI = i; }
        if((d != kD)) { m_pidControllerLeft.setD(d); kD = d; }
        if((iz != kIz)) { m_pidControllerLeft.setIZone(iz); kIz = iz; }
        if((ff != kFF)) { m_pidControllerLeft.setFF(ff); kFF = ff; }
        if((max != kMaxOutput) || (min != kMinOutput)) { 
            m_pidControllerLeft.setOutputRange(min, max); 
            kMinOutput = min; kMaxOutput = max; 
        }

        if((p != kP)) { m_pidControllerRight.setP(p); kP = p; }
        if((i != kI)) { m_pidControllerRight.setI(i); kI = i; }
        if((d != kD)) { m_pidControllerRight.setD(d); kD = d; }
        if((iz != kIz)) { m_pidControllerRight.setIZone(iz); kIz = iz; }
        if((ff != kFF)) { m_pidControllerRight.setFF(ff); kFF = ff; }
        if((max != kMaxOutput) || (min != kMinOutput)) { 
            m_pidControllerRight.setOutputRange(min, max); 
            kMinOutput = min; kMaxOutput = max; 
        }


    
        
        SmartDashboard.putNumber("LeftEncoder", m_encoderLeft.getVelocity());
        SmartDashboard.putNumber("RightEncoder", m_encoderRight.getVelocity());
        SmartDashboard.putNumber("JoystickValue", RobotContainer.getInstance().driverJoystick.getY());





        // smartdashboard

        // temps
        SmartDashboard.putNumber("LeftBackTemp", leftFollow.getMotorTemperature());
        SmartDashboard.putNumber("RightBackTemp", rightFollow.getMotorTemperature());
        SmartDashboard.putNumber("LeftFrontTemp", leftLeader.getMotorTemperature());
        SmartDashboard.putNumber("RightFrontTemp", rightLeader.getMotorTemperature());

        SmartDashboard.putNumber("LBMotorVoltage", leftFollow.getBusVoltage());
        SmartDashboard.putNumber("LFMotorVoltage", leftLeader.getBusVoltage());
        SmartDashboard.putNumber("RBMotorVoltage", rightFollow.getBusVoltage());
        SmartDashboard.putNumber("RFMotorVoltage", rightLeader.getBusVoltage());

        // encoder CPR

        SmartDashboard.putNumber("LeftBackPosition", encoderLeftFollow.getPosition());
        SmartDashboard.putNumber("RightBackPosition", encoderRightFollow.getPosition());
        SmartDashboard.putNumber("LeftFrontPosition", encoderLeftLeader.getPosition());
        SmartDashboard.putNumber("RightFrontPosition", encoderRightLeader.getPosition());

        SmartDashboard.putNumber("LBVelocity", encoderLeftFollow.getVelocity());
        SmartDashboard.putNumber("RBVelocity", encoderRightFollow.getVelocity());
        SmartDashboard.putNumber("LFVelocity", encoderLeftLeader.getVelocity());
        SmartDashboard.putNumber("RFVelocity", encoderRightLeader.getVelocity());

        // Pigeon2

        SmartDashboard.putNumber("Yaw", pigeon2.getYaw());
        SmartDashboard.putNumber("Pitch", pigeon2.getPitch());
        SmartDashboard.putNumber("Roll", pigeon2.getRoll());

        SmartDashboard.putNumber("leftSetPoint", setPointLeft);
        SmartDashboard.putNumber("rightSetPoint", setPointRight);


        // need yaw pitch and roll, to feed into the accelerometer
        // SmartDashboard.putNumber("acceleration",
        // pigeon2.getBiasedAccelerometer(null));

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void driveStop() {
        rightLeader.stopMotor();
        leftLeader.stopMotor();
    }

    public void maxSpeed() {
        driveArcade(1, 0);

    }
    //AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA (test)

    // lefty = power rightx = rotation
    public void driveArcade(double power, double rotation) {
        /*
         * double max = .9;
         * double rampUp = .001; // error allowed
         * double rampD = .001; // ramp down
         * double jsAxisY = rightX; // pull num from joystick
         * double sub = jsAxisY - oldAxisY;
         * 
         * if (Math.abs(sub) > rampUp && sub > 0) {
         * // test to see if it is going forward
         * tempAxisY = oldAxisY + rampUp;
         * } else if (Math.abs(sub) > rampD && sub < 0) { // test to see if it is going
         * backwards
         * tempAxisY = oldAxisY - rampD;
         * } else {
         * tempAxisY = jsAxisY;
         * }
         * // test to see if value exceds max allowed
         * 
         * if (tempAxisY < max * -1) {
         * tempAxisY = max * -1;
         * } else if (tempAxisY > max) {
         * tempAxisY = max;
         * }
         * finalAxisY = tempAxisY;
         * oldAxisY = finalAxisY;
         */
        finalAxisY = power;
        driveMain.arcadeDrive(finalAxisY, rotation);
    }


    public void driveVelocity(double power, double rotation) {
        power = MathUtil.clamp(power, -1.0, 1.0);
        rotation = MathUtil.clamp(rotation, -1.0, 1.0);
        
        double leftSpeed;
        double rightSpeed;
        
        //turn in place
        if (power == 0 && rotation != 0){
              leftSpeed = power - rotation;
              rightSpeed = power + rotation;
        } else {
              leftSpeed = power - Math.abs(power) * rotation;
              rightSpeed = power + Math.abs(power) * rotation;
        }
        
            // Desaturate wheel speeds
            double maxMagnitude = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
            if (maxMagnitude > 1.0) {
              leftSpeed /= maxMagnitude;
              rightSpeed /= maxMagnitude;
            }

        setPointLeft = rightSpeed * maxRPM;
        setPointRight = leftSpeed * maxRPM;
        m_pidControllerLeft.setReference(setPointLeft, CANSparkMax.ControlType.kVelocity);
        m_pidControllerRight.setReference(setPointRight, CANSparkMax.ControlType.kVelocity);

    }


    public void driveForwardSlow() {
        rightLeader.set(slowPower);

        leftLeader.set(slowPower);
    }

    public void driveBackwardSlow() {
        rightLeader.set(-slowPower);

        leftLeader.set(-slowPower);
    }

    public boolean isRobotBalanced() {
        if (pigeon2.getPitch() > balanceError && pigeon2.getPitch() < -balanceError) {
        isBalanced = true;
        } else isBalanced = false;
        return isBalanced;
    }

    
}
