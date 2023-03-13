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

import javax.lang.model.util.ElementScanner14;

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
private static WPI_VictorSPX rightAngler = new WPI_VictorSPX(24); //check this motor's CAN ID





//private Timer pivotTimer = new Timer();



private static volatile double targetAngle;


private static final double ARM_PIVOT_MAX_ANGLE = 183.0;   //Robot 0 deg = Arm pointing straight down
private static final double ARM_PIVOT_MIN_ANGLE = 77.0;     //TBD, the reason for 110 range is to give wiggle
                                                            // room for oscillations

private static double PIVOT_VOLTAGE_OFFSET = 0.0;//may change if the motors require higher voltage.
    // may need an offset if the motor voltage requirement is higher than the max limit of the analog input
    // in the case that an offset is used, when calculating the angle, add back the offset to value



public AnalogInput armPivotEnc;
private static final int    ARM_PIVOT_ENCODER_ANALOG_PORT = 0;
private static final double ARM_PIVOT_ENC_MAX_VOLTAGE     = 4.784;


public static boolean armTargetHit = false;
public static boolean up = false;
public static boolean down = false;
private static double command = 0.0;

private static final double INPUT_THRESHOLD = 1.0E-3;

public BananaArm(){

    armPivotEnc = new AnalogInput(ARM_PIVOT_ENCODER_ANALOG_PORT);

            
    }


    public static void testMotorsUp()
    {
        
        leftAngler.set(command -= Robot.controller2.getRightTriggerAxis() * 0.05);
        rightAngler.set(command -= Robot.controller2.getRightTriggerAxis() * 0.05);
        
    }

    public static void testMotorsDown()
    {
        leftAngler.set(command += Robot.controller2.getLeftTriggerAxis() * 0.05);
        rightAngler.set(command += Robot.controller2.getLeftTriggerAxis() * 0.05);
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

    public void increaseTargetAngle()
    {
        targetAngle+=5;
    }

    public void decreaseTargetAngle()
    {
        targetAngle-=5; //smaller because gravity pulls the arm that is already trying to go down
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
        Thread t = new Thread(() ->
        {
            final double ARM_PIVOT_THREAD_WAITING_TIME = 0.005;
            final double kP = 0.021;//0.020 //0.013; //TODO
            final double kD = 0.0012;//0.00020
            final double kI = 0.0;
            final double kA = 0.34;//0.38//0.0077;
            final double kF = 0.0;//-0.05;

            double power;            
            double kPpower;
            double kIpower;
            double kDpower;
            double kApower;
            double kFpower;

            Timer armTimer = new Timer();
            armTimer.start();

            double previousError = 0;
            double currentError; 
            double deltaError = 0; 

            double previousDerivative = 0;
            double currentDerivative;    
            double filteredDerivative;  // filtered to prevent derivative jumps, and provide 
                                        //a smoother transition to a new slope of the dE v. dt graph
            
            double previousTime = 0;
            
            double deltaTime;
            
            double currentTime;
            double currentAngle;

            double integral = 0;

            while(true)
            {
                if(targetAngle == BananaConstants.INVALID_ANGLE)
                {
                    Timer.delay(BananaConstants.CONTROLLER_INPUT_WAIT_TIME);
                }
                else
                {
                    //SmartDashboard.putBoolean("pivot pid state", runPivotPID);
                    currentTime  = armTimer.get();
                    currentAngle = getPivotAngle();

                    currentError = targetAngle - currentAngle;
                    
                    /*if(Math.abs(currentError-previousError) < 1.0)
                    {
                       if((armTimer.get() - currentTime) > 0.1)
                       {
                            runPivotPID = false;
                            turnPivot(0.0);
                            setPivotTargetAngle(BananaConstants.INVALID_ANGLE);
                            Thread.currentThread().interrupt();
                       }    
                          
                    }*/

                    //if(runPivotPID == true)
                    //{
                        
                        deltaError = currentError - previousError;
                        deltaTime  = currentTime  - previousTime;

                        integral += deltaTime * currentError;

                        currentDerivative = (deltaError / deltaTime);
                        //filteredDerivative = (0.7 * currentDerivative) + (0.3 * previousDerivative);


                        kPpower = kP * currentError;
                        kIpower = kI * integral;
                        kDpower = kD * currentDerivative;//filteredDerivative;//currentDerivative;//filteredDerivative
                        kApower = (kA * (Math.cos(Math.toRadians(159.5 - currentAngle))));
                        kFpower = kF;

                        power = kPpower + kIpower + kDpower + kApower + kFpower;
                        //power = (kP * currentError) + (kA * (Math.cos(Math.toRadians(currentAngle - 324.5))));//ka compensates for angle of arm
                                //arm extension distance + 13 is the distance from pivot to wrist
                                //(kD * currentDerivative) + kF; //+ (kI * integral)

                        
                        pivotArm(-power);
                        //armPivotMtrCtrlRT.set(power);
                    
                        previousError = currentError;
                        previousTime = currentTime;

                        previousDerivative = currentDerivative;

                        SmartDashboard.putNumber("P Power", kPpower);
                        SmartDashboard.putNumber("I Power", kIpower);
                        SmartDashboard.putNumber("D Power", kDpower);
                        SmartDashboard.putNumber("A Power", kApower);
                        SmartDashboard.putNumber("F Power", kFpower);
                        SmartDashboard.putNumber("Total Arm Power", power);
                    
                        Timer.delay(ARM_PIVOT_THREAD_WAITING_TIME);
                    //}
                }
            }
        });
        t.start();
    }

    /*public void pivotArm2(double power)
    {

        double pivotAngle = this.getPivotAngle();

        if(Math.abs(power) <= INPUT_THRESHOLD)
            {
                setPivotTargetAngle(pivotAngle);
                power = 0;
            }
            
        if (power > 1)
        {
            power = 1;
        }
        if (power < -1)
        {
            power = -1;
        }

        if(Math.abs(power) >= INPUT_THRESHOLD)
            {      
                leftAngler.set(-power);
                rightAngler.set(-power);
            }
            else
            {
                leftAngler.set(0);
                rightAngler.set(0);
            }  
    }

    */
    public void pivotArm(double power){
        //using limits

        if (power > 0.9)
        {
            power = 0.9;
        }
        if (power < -0.9)
        {
            power = -0.9;
        }

        double pivotAngle = this.getPivotAngle();

        if(Math.abs(power) <= INPUT_THRESHOLD)
            {
                //setPivotTargetAngle(pivotAngle);
            }
            else if (power > 0.0)
            {
                //setPivotTargetAngle(BananaConstants.INVALID_ANGLE);

                if (pivotAngle > ARM_PIVOT_MAX_ANGLE || Math.abs(power) > 1.0)
                {
                    leftAngler.set(0.0);
                    rightAngler.set(0.0);
                    setPivotTargetAngle(ARM_PIVOT_MAX_ANGLE); //CHANGED HERE
                }
                    else //if (Math.abs(power) > INPUT_THRESHOLD)
                    {
                        leftAngler.set(power);
                        rightAngler.set(power); //rotate arm clockwise which means up
                    }
            }
            else if (power < 0.0)
            {
                //setPivotTargetAngle(BananaConstants.INVALID_ANGLE);

                if (pivotAngle < ARM_PIVOT_MIN_ANGLE || Math.abs(power) > 1.0)
                {
                    leftAngler.set(0.0);
                    rightAngler.set(0.0);
                    setPivotTargetAngle(ARM_PIVOT_MIN_ANGLE);
                }
                    else //if ( Math.abs(power) > INPUT_THRESHOLD)
                    {
                        leftAngler.set(power);
                        rightAngler.set(power); //rotate arm counterclockwise which means down
                    }
                    
                

                
            }
            

    }

    public double getArmTemp(int id)
    {
        double temp = -999.0;
        switch (id)
        {

            case 24:
                temp = rightAngler.getTemperature();
            break;

            case 28:
                temp = leftAngler.getTemperature();
            break;

            default :
                temp = -999.0;          
        }
        return temp; 
    }
}