package frc.robot.Mechanisms;

//import com.ctre.phoenix.motorcontrol.FeedbackDevice;
//import com.ctre.phoenix.motorcontrol.NeutralMode;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;



//import edu.wpi.first.networktables.NetworkTable;
//import edu.wpi.first.networktables.NetworkTableEntry;
//import edu.wpi.first.networktables.NetworkTableInstance;

//import edu.wpi.first.wpilibj.TimedRobot;
//import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class BananaDriveTrain {
    private CANSparkMax Rdrive1;
    private CANSparkMax Rdrive2;
    private CANSparkMax Ldrive1; 
    private CANSparkMax Ldrive2;

    private static int Rdrive1ID = 21;
    private static int Rdrive2ID = 22;
    private static int Ldrive1ID = 20;
    private static int Ldrive2ID = 23;

    private static DifferentialDrive drivebase;

    public double left_command = 0;
    public double right_command = 0;
    
    //private final double SPARKMAX_INTEGRATED_ENC_CNTS_PER_REV      = 2048.0;
    //private final double DRVTRAIN_WHEEL_RADIUS                    = 2;
    //private final double DRVTRAIN_WHEEL_CIRCUMFERENCE             = (2.0 * Math.PI * DRVTRAIN_WHEEL_RADIUS);

    public boolean aimPIDState = false;

    public  double         currentEncCountsToInches = 0.0;
 
    //private Thread turnThread;
 
 
    /*----------------------------------------------------------------------------------------------
    *  Autonomous Closed Loop Control - Velocity
    *---------------------------------------------------------------------------------------------*/
    //private final int DRVTRAIN_VELOCITY_PID_IDX = 0;
    //private final int PID_TIMEOUT_MS            = 10;

    public final double RT_PID_P = 0.04;  
    public final double RT_PID_I = 0.0; 
    public final double RT_PID_D = 0.0;    
    public final double RT_PID_F = 1080.0/20480.0; 


    //private      double drvStraightTargetVelocityOffsetFwd = 50.0;
    
    public static boolean turnTargetHit = false;
 
    private Thread coneThread;
    private Thread cubeThread;
 
    public BananaDriveTrain()
    {
       

        
        Rdrive1 = new CANSparkMax(Rdrive1ID, MotorType.kBrushless);
        Rdrive2 = new CANSparkMax(Rdrive2ID, MotorType.kBrushless);
        Ldrive1 = new CANSparkMax(Ldrive1ID, MotorType.kBrushless); 
        Ldrive2 = new CANSparkMax(Ldrive2ID, MotorType.kBrushless);

        MotorControllerGroup RIGHT = new MotorControllerGroup(Rdrive1, Rdrive2);
        MotorControllerGroup LEFT = new MotorControllerGroup(Ldrive1, Ldrive2);

       drivebase = new DifferentialDrive(LEFT, RIGHT); 

       Rdrive1.restoreFactoryDefaults();
       Rdrive2.restoreFactoryDefaults();
       Ldrive1.restoreFactoryDefaults();
       Ldrive2.restoreFactoryDefaults();
      
       //BACK MOTORS FOLLOW FRONT MOTORS
       Rdrive2.follow(Rdrive1);
       Ldrive2.follow(Ldrive1);
     
       RIGHT.setInverted(true); 

     
        


     
       //setDriveTrainPIDConfiguration(LEFT, LT_PID_P, LT_PID_I, LT_PID_D, LT_PID_F); //TBD Only used for autonomous
       //setDriveTrainPIDConfiguration(RIGHT, RT_PID_P, RT_PID_I, RT_PID_D, RT_PID_F);
     
      
    }
     
      
       
     /*----------------------------------------------------------------------------------------------
    *
    *  Motor Config & Status Methods
    *
    *---------------------------------------------------------------------------------------------*/
    public void setToBrakeMode()
    {
        Ldrive1.setIdleMode(IdleMode.kBrake); 
        Ldrive2.setIdleMode(IdleMode.kBrake);
        Rdrive1.setIdleMode(IdleMode.kBrake);
        Rdrive2.setIdleMode(IdleMode.kBrake);
    }

    public void setToCoastMode() 
    {
        Ldrive1.setIdleMode(IdleMode.kCoast); 
        Ldrive2.setIdleMode(IdleMode.kCoast);
        Rdrive1.setIdleMode(IdleMode.kCoast);
        Rdrive2.setIdleMode(IdleMode.kCoast);
    }

    
    public static void tankDrive(double L, double R){

        drivebase.tankDrive(-L*0.3, -R*0.3);  //set to half speed for now, may change for competition

    }


    //For auton, maybe do PID turn

    /*----------------------------------------------------------------------------------------------
    *
    *  Closed Loop Control Methods
    *
    *---------------------------------------------------------------------------------------------*/

    public void coneAimPID(){
        coneThread = new Thread(() ->
    {
      float kAimP = -0.01f;  //may need to calibrate kAimP or min_command if aiming causes occilation
      float min_command = 0.0005f;
  
      float cone_heading_error;
      float steering_adjust;
      
      float limeX = (float) SmartDashboard.getNumber("tx", 0.0f);
      double validTarget = SmartDashboard.getNumber("tv", 0.0);  

      

  
      while (true)
      {
        if(aimPIDState==false || validTarget == 0.0)
        {
            break;
        }

            else if(aimPIDState==true && validTarget == 1.0)
            {

         
            cone_heading_error = -(limeX); //figure out what x values limelight produces for the target in order to scale kAimP
            steering_adjust = 0.0f;
  
                if (Math.abs(cone_heading_error) > 1.0)
                {
                  if (cone_heading_error < 0)
                  {
                          steering_adjust = kAimP*cone_heading_error + min_command;
                  }
                  else
                  {
                          steering_adjust = kAimP*cone_heading_error - min_command;
                  }
                }
            left_command += steering_adjust;
            right_command -= steering_adjust;

                //Stop motors if target hit within close accuracy
            if (Math.abs(cone_heading_error) < 1.0)
            {
            left_command = 0;
            right_command = 0;
            drivebase.tankDrive(0, 0);
            }

            aimBot(left_command, right_command);
        
            }   
      }

      
    });
        coneThread.start();
    }
    

    
    

    public void cubeAimPID(){

        cubeThread = new Thread(() ->
    {
      float kAimP = -0.000005f;  //may need to calibrate kAimP or min_command if aiming causes occilation
      float min_command = 0.000005f;
  
      float cube_heading_error;
      float steering_adjust;

      float aprilX = (float) SmartDashboard.getNumber("Center X", 0.0f);
      double validTarget = SmartDashboard.getNumber("tv", 0.0);



      while (true)                                  //checks that we are in aiming mode AND limelight 
                                                    //isn't focusing on a reflective target so that
                                                    //we dont have two aimPID's running simultaneously

                                                    //MAY HAVE TO REVISE HOW TO DISTINGUISH CONE V. CUBE
                                                    //AIM PIDS
      {
        if(aimPIDState==false || validTarget==1.0)
        {
            break;
        }

            else if(aimPIDState==true && validTarget == 0.0)
            {

            //Trying to make the heading error 0 relative to the center of the usb camera display
            cube_heading_error = -(aprilX+320);//half the width (in pixels) of the camera display
            steering_adjust = 0.0f;
  
            if (Math.abs(cube_heading_error) > 1.0)
            {
                if (cube_heading_error < 0)
                {
                    steering_adjust = kAimP*cube_heading_error + min_command;
                }
                  else
                  {
                    steering_adjust = kAimP*cube_heading_error - min_command;
                  }
            }
        left_command += steering_adjust;
        right_command -= steering_adjust;

        //Stop motors if target hit within close accuracy (10 pixels)
        if (Math.abs(cube_heading_error) < 15.0){
            left_command = 0;
            right_command = 0;
        }

        aimBot(left_command, right_command);

        }

      }

      
    });
        cubeThread.start();

    }

    public void aimBot(double left_command, double right_command)
    {
        if(Math.abs(left_command) < 0.01 && Math.abs(right_command) < 0.01)
        {
            drivebase.tankDrive(0, 0);
        }
            else if(Math.abs(left_command) <= 1 && Math.abs(right_command) <= 1)
            {
                drivebase.tankDrive(left_command, right_command);
            }
    }


    public static void balance()
    {
        if (BananaBrake.isBrakeOn() == true){
            
        }
    }


    
    /*----------------------------------------------------------------------------------------------
    *
    *  Miscellaneous Methods
    *
    *---------------------------------------------------------------------------------------------*/


     public double getMotorTemperature(int id)
    {
        double temp = -999.0;
        switch (id)
        {
            case 21:
                temp = Rdrive1.getMotorTemperature();
            break;

            case 22:
                temp = Rdrive2.getMotorTemperature();
            break;

            case 20:
                temp = Ldrive1.getMotorTemperature();
            break;
                
            case 23:
                temp = Ldrive2.getMotorTemperature();
            break;

            default :
                temp = -999.0;          
        }
        return temp; 
    }
}
