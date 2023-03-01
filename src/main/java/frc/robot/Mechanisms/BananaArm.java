
/*
 *  Author : Alex Naehu
 * 
 * Functionality : controls the arm using 
 * 
 *  Methods :  controls the arm extension by the power, controls the arm pivot by the power,
 *             gets the status of each limit switch, gets the angle of the arm pivot,  
 *             moves the arm extension to the target distance, moves the arm pivot to the targetAngle
 * 
 *  Revision History : First created 1/13/23
 * 
 * 
 */
package frc.robot.Mechanisms;

//import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj.XboxController;

//import com.ctre.phoenix.motorcontrol.can.BaseMotorController; //used to make VictorSPX motors follow eachother
//import com.ctre.phoenix.motorcontrol.FeedbackDevice;
//import com.ctre.phoenix.motorcontrol.IMotorController;
//import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import frc.robot.Robot.BananaConstants;
import frc.robot.Robot.*;
import edu.wpi.first.wpilibj.AnalogInput;



public class BananaArm{

private static WPI_VictorSPX leftAngler = new WPI_VictorSPX(28); //made both of these static
private static WPI_VictorSPX rightAngler = new WPI_VictorSPX(29);




private Thread pivotThread;

//private Timer pivotTimer = new Timer();



private static volatile double targetAngle;
private static final double ARM_PIVOT_MAX_ANGLE = 110.0;   //Robot 0 deg = Arm pointing straight down
private static final double ARM_PIVOT_MIN_ANGLE = 0.0;     //TBD, the reason for 100 is so an error of 
                                                            //10 degrees overshoot wont break the code

private static double PIVOT_VOLTAGE_OFFSET = 0.0;//may change if the motors require higher voltage.
    // may need an offset if the motor voltage requirement is higher than the max limit of the analog input
    // in the case that an offset is used, when calculating the angle, add back the offset to value



public static AnalogInput armPivotEnc;
private static final int    ARM_PIVOT_ENCODER_ANALOG_PORT = 0;
private static final double ARM_PIVOT_ENC_MAX_VOLTAGE     = 4.784;


public static boolean armTargetHit = false;


public BananaArm(){

    armPivotEnc = new AnalogInput(ARM_PIVOT_ENCODER_ANALOG_PORT);

            
    }


    public static void testMotorsUp()
    {
        leftAngler.set(-0.2);
        rightAngler.set(-0.2);
    }

    public static void testMotorsDown()
    {
        leftAngler.set(0.2);
        rightAngler.set(0.2);
    }

    public double getPivotAngle() 
    {   
        return (((armPivotEnc.getVoltage() + PIVOT_VOLTAGE_OFFSET) / ARM_PIVOT_ENC_MAX_VOLTAGE) * 360.0);
    }
  
    public double getPivotTargetAngle()
    {
        return targetAngle;
    }

    public void setPivotTargetAngle(double angle)
    {
        targetAngle = angle;
    }

    public static void increaseTargetAngle()
    {
        targetAngle+=0.1;
    }

    public static void decreaseTargetAngle()
    {
        targetAngle-=0.1;
    }

    public void setArmTargetHit(boolean state)
    {
        armTargetHit = state;
    }

    public boolean getArmTargetHit()
    {
        return armTargetHit;
    }


    public static void setPosition(double pivotTargetAngle)
    {
        Robot.arm.setPivotTargetAngle(pivotTargetAngle);

        Robot.arm.setArmTargetHit(false);
    }

    
    public void pivotPID()
    {
        pivotThread = new Thread(() ->
        {
            final double ARM_PIVOT_THREAD_WAITING_TIME = 0.005;
            final double kP = 0.005;//0.007
            final double kD = 0.0005; 
            final double kI = 0.0000;//0.00001
            //final double kA = 0.0022;//0.0077;
            //final double kF = 0.0;//-0.05;

            double power;            

            Timer pivotTimer = new Timer();
            pivotTimer.start();

            double previousError = 0;
            double currentError; 
            double deltaError = 2; 

            double previousDerivative = 0;
            double currentDerivative;    // in case you want to filter derivative
            double filteredDerivative;
            
            double previousTime = 0;
            
            double deltaTime;
            
            double currentTime;
            double currentAngle;

            double integral = 0;

            double kAPow;
            double kPPow;
            double kDPow;
            double kIPow;
            
            boolean runPivotPID;
            
            armTargetHit = false;
            

            while(armTargetHit == false && getPivotAngle() < ARM_PIVOT_MAX_ANGLE && getPivotAngle() > ARM_PIVOT_MIN_ANGLE)
            {
               
                    runPivotPID = true;
                
                    SmartDashboard.putBoolean("pivot pid state", runPivotPID);
                    currentTime = pivotTimer.get();
                    currentAngle = getPivotAngle();

                    currentError = targetAngle - currentAngle;
                    
                    //accuracy checker (within 1 degree of goal)
                    if(Math.abs(deltaError) < 1.0)
                    {
                       if((pivotTimer.get() - currentTime) > 0.1)
                       {
                            runPivotPID = false;
                            leftAngler.set(0.0);
                            rightAngler.set(0.0);
                            Thread.currentThread().interrupt();
                       }    
                          
                    }

                    if(runPivotPID == true)
                    {
                        deltaError = currentError - previousError;
                        deltaTime = currentTime - previousTime;

                        integral += deltaTime * currentError;

                        currentDerivative = (deltaError / deltaTime);
                        //might change filtering
                        filteredDerivative = (0.7 * currentDerivative) + (0.3 * previousDerivative);

                        kAPow = 0; //kA * ((getArmExtensionDistance() + 13) * Math.cos(Math.toRadians(currentAngle - 45)));
                        kPPow = kP * currentError;
                        kDPow = kD * filteredDerivative; //currentDerivative
                        kIPow = kI * integral;

                        SmartDashboard.putNumber("ka pow", kAPow);
                        SmartDashboard.putNumber("kp pow", kPPow);
                        SmartDashboard.putNumber("kd pow", kDPow);
                        SmartDashboard.putNumber("ki pow", kIPow);

                        power = kPPow + kIPow + kDPow + kAPow;//ka compensates for angle of arm
                                //arm extension distance + 13 is the distance from pivot to wrist
                                // + kF; //+ (kI * integral)

                        SmartDashboard.putNumber("Arm Power", power);

                        //should probably make angler 2 follow angler 1 somehow
                        leftAngler.set(-power);
                        rightAngler.set(-power);

                        SmartDashboard.putNumber("pivot error", currentError);
                    
                        previousError = currentError;
                        previousTime = currentTime;

                        previousDerivative = currentDerivative;
                    
                        Timer.delay(ARM_PIVOT_THREAD_WAITING_TIME);
                    }
                        if(getPivotAngle()==getPivotTargetAngle()){
                            //leftAngler.set(0);
                            //rightAngler.set(0);
                            armTargetHit = true;
                        }
            }
                
        });
        pivotThread.setDaemon(true);
        pivotThread.start();
        
    }
}
