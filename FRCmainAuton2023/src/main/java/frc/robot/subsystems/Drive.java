package frc.robot.subsystems;

import java.beans.MethodDescriptor;
import java.lang.reflect.Method;

import org.ejml.dense.row.decomposition.eig.watched.WatchedDoubleStepQREigen_DDRM;

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
import frc.robot.commands.DriveStop;
import frc.robot.commands.driveBalance;
import frc.robot.commands.driveTurnDegree;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;

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

    public SparkMaxPIDController m_pidControllerLeft;
    public SparkMaxPIDController m_pidControllerRight;
    public SparkMaxPIDController m_pidControllerLeftAuton;
    public SparkMaxPIDController m_pidControllerRightAuton;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;
    public double kPAuton, kIAuton, kDAuton, kIzAuton, kFFAuton, kMaxOutputAuton, kMinOutputAuton, maxRPMAuton;
    

    private double balanceError = 1;
    private double slowPower = 0.15;
    private boolean isBalanced = false;
    private  double setPointLeft;
    private double setPointRight; 

    private double deadband = 0.1;

    double targetPositionL;
    double targetPositionR;

    double turnTarget;
    double turnTargetFinal;
    private double turnError = 50;
    private double turnSpeed = 0.9;
    private double intialYaw;


    /**
    *
    */
    public Drive() {

        

        rightLeader = new CANSparkMax(3, MotorType.kBrushless);
        rightLeader.restoreFactoryDefaults();
        rightLeader.setIdleMode(IdleMode.kCoast);
        rightLeader.setSmartCurrentLimit(40);
        rightLeader.setInverted(false);

        leftLeader = new CANSparkMax(2, MotorType.kBrushless);
        leftLeader.restoreFactoryDefaults();
        leftLeader.setIdleMode(IdleMode.kCoast);
        leftLeader.setSmartCurrentLimit(40);
        leftLeader.setInverted(true);

        driveMain = new DifferentialDrive(leftLeader, rightLeader);
        addChild("DriveMain", driveMain);
        driveMain.setSafetyEnabled(true);
        driveMain.setExpiration(0.1);
        driveMain.setMaxOutput(1.0);

        rightFollow = new CANSparkMax(4, MotorType.kBrushless);
        rightFollow.restoreFactoryDefaults();
        // rightFollow.setInverted(false);
        rightFollow.setIdleMode(IdleMode.kCoast);
        rightFollow.setSmartCurrentLimit(40);
        rightFollow.follow(rightLeader, false);

        leftFollow = new CANSparkMax(1, MotorType.kBrushless);
        leftFollow.restoreFactoryDefaults();
        leftFollow.setIdleMode(IdleMode.kCoast);
        leftFollow.setSmartCurrentLimit(40);
        leftFollow.follow(leftLeader, false); // Same direction as leader




        //create pid
        m_pidControllerLeft = leftLeader.getPIDController();
        m_pidControllerRight = rightLeader.getPIDController();

        m_pidControllerLeftAuton = leftLeader.getPIDController();
        m_pidControllerRightAuton = rightLeader.getPIDController();

        setDrivePID();
        setDrivePIDAuton();






        pigeon2 = new Pigeon2(19);
        


        // sets the two follow motors to follow the lead motors       


        encoderLeftFollow = leftFollow.getEncoder(Type.kHallSensor, 42);
        encoderLeftFollow.setVelocityConversionFactor(1);
        encoderLeftFollow.setPositionConversionFactor(42);

        encoderRightFollow = rightFollow.getEncoder(Type.kHallSensor, 42);
        encoderRightFollow.setVelocityConversionFactor(1);
        encoderRightFollow.setPositionConversionFactor(42);

        encoderLeftLeader = leftLeader.getEncoder(Type.kHallSensor, 42);
        encoderLeftLeader.setVelocityConversionFactor(1);
        encoderLeftLeader.setPositionConversionFactor(42);

        encoderRightLeader = rightLeader.getEncoder(Type.kHallSensor, 42);
        encoderRightLeader.setVelocityConversionFactor(1);
        encoderRightLeader.setPositionConversionFactor(42);


        // set PID coefficients


        // display PID coefficients on SmartDashboard
        SmartDashboard.putNumber("P Gain", kP);
        SmartDashboard.putNumber("I Gain", kI);
        SmartDashboard.putNumber("D Gain", kD);
        SmartDashboard.putNumber("I Zone", kIz);
        SmartDashboard.putNumber("Feed Forward", kFF);
        SmartDashboard.putNumber("Max Output", kMaxOutput);
        SmartDashboard.putNumber("Min Output", kMinOutput);

        SmartDashboard.putNumber("targetPostionL", targetPositionL);

        
        

       
  }

  

    @Override
    public void periodic() {

        //
        //
        driveMain.setExpiration(1);
        driveMain.feed();
      
        // DO NOT MOVE, OR DELETE THIS LINE OF CODE!!!!!!!!!!!!!!!
        //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


        /*
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
        /*if((p != kP)) { m_pidControllerLeft.setP(p); kP = p; }
        ((i != kI)) { m_pidControllerLeft.setI(i); kI = i; }
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
        */



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

        SmartDashboard.putNumber("setPointRight", setPointRight);
        SmartDashboard.putNumber("setPointLeft", setPointLeft);

        SmartDashboard.putBoolean("IsRoboBalanced", isRobotBalanced());

        SmartDashboard.putNumber("TargetPosLeft", targetPositionL);
        SmartDashboard.putNumber("TargetPosRight", targetPositionR);
        
        SmartDashboard.putNumber("TurnInial", intialYaw);
        SmartDashboard.putNumber("turnTarget", turnTarget);
        SmartDashboard.putNumber("tunnTargetFinal", turnTargetFinal);
        SmartDashboard.putNumber("CountsPerRev", encoderLeftLeader.getCountsPerRevolution());

        

        
        


        


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
        driveVelocity(-1, 0);

    }
    //AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA (test)

    // lefty = power rightx = rotation
   /* public void driveArcade(double power, double rotation) {
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
         
        finalAxisY = power;
        SmartDashboard.putNumber("finalaxisy", finalAxisY);
        SmartDashboard.putNumber("rotation", rotation);
        driveMain.arcadeDrive(finalAxisY, rotation);
    }
    */
    public void driveZero(){
        driveMain.arcadeDrive(0, 0);
    }
    

    public void setDrivePID(){
        kP = 0.00006;
        kI = 0.0000006;
        kD = 0; 
        kIz = 0; 
        kFF = 0.000015;  //.00015 default
        kMaxOutput = 1; 
        kMinOutput = -1;
        maxRPM = 4500; //max rpm (goal) 
         

        m_pidControllerLeft.setP(kP);
        m_pidControllerLeft.setI(kI);
        m_pidControllerLeft.setD(kD);
        m_pidControllerLeft.setIZone(kIz);
        m_pidControllerLeft.setFF(kFF);
        m_pidControllerLeft.setOutputRange(kMinOutput, kMaxOutput);

       m_pidControllerLeft.setIMaxAccum(.8, 0);
       

        m_pidControllerRight.setP(kP);
        m_pidControllerRight.setI(kI);
        m_pidControllerRight.setD(kD);
        m_pidControllerRight.setIZone(kIz);
        m_pidControllerRight.setFF(kFF);
        m_pidControllerRight.setOutputRange(kMinOutput, kMaxOutput);

        m_pidControllerRight.setIMaxAccum(.8, 0);

    }

    public void setDrivePIDAuton(){
        kPAuton = 0.00045; //  0.0004;
        kIAuton = 0.00000000000000005; //// defult  0.00000000000000000005
        kDAuton = 0.0; 
        kIzAuton = 0; 
        kFFAuton = 0.00;  //.0.00 default
        kMaxOutputAuton = .3; //-.3
        kMinOutputAuton = -.3;//-.3
        maxRPMAuton = 500; //max rpm (goal) 500
         
        m_pidControllerLeftAuton.setP(kPAuton);
        m_pidControllerLeftAuton.setI(kIAuton);
        m_pidControllerLeftAuton.setD(kDAuton);
        m_pidControllerLeftAuton.setIZone(kIzAuton);
        m_pidControllerLeftAuton.setFF(kFFAuton);
        m_pidControllerLeftAuton.setOutputRange(kMinOutputAuton, kMaxOutputAuton);

        m_pidControllerLeftAuton.setIMaxAccum(.2, 0);

        m_pidControllerRightAuton.setP(kPAuton);
        m_pidControllerRightAuton.setI(kIAuton);
        m_pidControllerRightAuton.setD(kDAuton);
        m_pidControllerRightAuton.setIZone(kIzAuton);
        m_pidControllerRightAuton.setFF(kFFAuton);
        m_pidControllerRightAuton.setOutputRange(kMinOutputAuton, kMaxOutputAuton);

        m_pidControllerRightAuton.setIMaxAccum(.2, 0);

    }

    public void driveVelocity(double power, double rotation) {
        

         //adds a deadzone 
         if(Math.abs(power) <= deadband){
            power = 0;
        }
        if(Math.abs(rotation) <= deadband){
            rotation = 0;
        }
        
        power = MathUtil.clamp(power, -1.0, 1.0);
        rotation = MathUtil.clamp(rotation, -1.0, 1.0);

         //Madelena ramp up and down
            
         double jsAxisY = power; // pull num from joystick

         double rampUpForward = .01; // error allowed
         double rampDownForward = .01; // ramp down
         double subForward = jsAxisY - oldAxisY;

         double rampUpBackward = -.025; // error allowed
         double rampDownBackward = -.015; // ramp down 
         double subBackward = jsAxisY - oldAxisY;

         if(oldAxisY > 0){ // test if going forward
            // test to see if oldAxis is less than .25 && it is accelerating 
            if(oldAxisY < .25 && Math.abs(subForward) > rampUpForward && subForward > 0){
                tempAxisY = oldAxisY + rampUpForward;
            
            } else if(oldAxisY < .25 && Math.abs(subForward) > rampDownForward && subForward < 0){
                tempAxisY = oldAxisY - rampDownForward;
            }else{
                tempAxisY = jsAxisY;
            }

         }else if(oldAxisY < 0){ // test if going backwards

            if(oldAxisY > -1 && Math.abs(subBackward) > Math.abs(rampUpBackward) && subBackward < 0){
                tempAxisY = oldAxisY + rampUpBackward;
            //
            } else if(oldAxisY > -1 && Math.abs(subBackward) > Math.abs(rampDownBackward) && subBackward > 0){
                tempAxisY = oldAxisY - rampDownBackward;
            }else{
                tempAxisY = jsAxisY;
            } 
         }else{ // standing still
            tempAxisY = jsAxisY;
         }
         
         power = tempAxisY;
         oldAxisY = power;
         
        
// end of Madelena's
        
        double leftSpeed;
        double rightSpeed;
        
        //turn in place
        if (power == 0 && rotation != 0){
              leftSpeed = power - rotation*.5;
              rightSpeed = power + rotation*.5;
        } else {

            
            leftSpeed = power - (Math.abs(power) * rotation)*.33;
            rightSpeed = power + (Math.abs(power) * rotation)*.33;

            
        }
        
            // Desaturate wheel speeds
            double maxMagnitude = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
            if (maxMagnitude > 1.0) {
              leftSpeed /= maxMagnitude;
              rightSpeed /= maxMagnitude;
            }


        setPointLeft = leftSpeed * maxRPM;
        setPointRight = rightSpeed * maxRPM;

        m_pidControllerLeft.setReference(setPointLeft, CANSparkMax.ControlType.kVelocity);
        m_pidControllerRight.setReference(setPointRight, CANSparkMax.ControlType.kVelocity);

    }




     public void driveForwardSlow() {
        driveVelocity(slowPower, 0);
    }

    public void driveBackwardSlow() {
        driveVelocity(-slowPower, 0);
    }



    public boolean isRobotBalanced() {
        if (pigeon2.getPitch() < balanceError && pigeon2.getPitch() > -balanceError) {
        isBalanced = true;
        } else isBalanced = false;
        return isBalanced;
    }

    public void balanceDrive(){
        SmartDashboard.putString("TEXTBalanceDrive", "Started");
        if(pigeon2.getPitch() < (-balanceError)){
            SmartDashboard.putString("TEXTBalanceDrive", "forward");
            driveDistance(5);
            
        }
        if(pigeon2.getPitch() > (balanceError)){
            SmartDashboard.putString("TEXTBalanceDrive", "BackWard");
            driveDistance(-5);
           
        }

    }

    

     public void driveDistance(double numberOfInches){
        setDrivePIDAuton();

        double TICKS_PER_INCH = (10.75*42)/(6*Math.PI);
        SmartDashboard.putNumber("TICKS_PER_INCH", TICKS_PER_INCH);
        double numberOfTicks = (TICKS_PER_INCH * numberOfInches); 

        SmartDashboard.putNumber("numberOFTicks", numberOfTicks);
       
        targetPositionL = encoderLeftLeader.getPosition() + numberOfTicks;
        targetPositionR = encoderRightLeader.getPosition() + numberOfTicks;

        m_pidControllerLeftAuton.setReference(targetPositionL, CANSparkMax.ControlType.kPosition);
        m_pidControllerRightAuton.setReference(targetPositionR, CANSparkMax.ControlType.kPosition);
    } 

    public boolean distanceDone(){
        double positionDelta = 10;
        boolean isDistanceDone = false;

    if((encoderLeftLeader.getPosition() <= (targetPositionL+positionDelta)) 
    && (encoderLeftLeader.getPosition() >= (targetPositionL-positionDelta))) {
        isDistanceDone = true;
    }
    return isDistanceDone;



    }

    public void driveTurnDegree(double degree){
        turnTarget = degree;
        intialYaw = pigeon2.getYaw();
      
            driveVelocity(0, (turnSpeed)*-1);
    }


    public boolean turnDone(){
        turnTargetFinal = intialYaw + turnTarget;
        if(Math.abs(pigeon2.getYaw()-turnTargetFinal) <= turnError){
            return true;
        }
        return false;
    }

    public void resetIAccum(){
        m_pidControllerLeft.setIAccum(0);
        m_pidControllerRight.setIAccum(0);
    }


    


}
