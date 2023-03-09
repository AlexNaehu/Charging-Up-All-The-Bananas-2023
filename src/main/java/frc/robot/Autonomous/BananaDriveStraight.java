package frc.robot.Autonomous;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Mechanisms.BananaDriveTrain;
import frc.robot.Robot.BananaConstants;
import frc.robot.Robot.Robot;

public class BananaDriveStraight {
    
    private static Timer clock = new Timer();
    



    public static void driveStraight(int distance)
    {

        Timer.delay(BananaConstants.NAVX_RESET_WAIT_TIME);
        
        clock.reset();
        clock.start();
        
        switch (distance){

            case(0):    //straightForwardSide(0)
                Robot.driveTrain.tankDrive(-0.5, -0.5);
                if(clock.get() > 3)
                {
                    Robot.driveTrain.tankDrive(0,0);
                }
            break;

            case(1):    //straightForwardMid(1)
                Robot.driveTrain.tankDrive(-0.5, -0.5);
                if(clock.get() > 2)
                {
                    Robot.driveTrain.tankDrive(0,0);
                }
            break;

            case(2):    //tipToeForward(2)
                Robot.driveTrain.tankDrive(-0.3, -0.3);
                if(clock.get() > 1.0)
                {
                    Robot.driveTrain.tankDrive(0,0);
                }
            break;

            case(3):    //straightBackwardSide(3)
                Robot.driveTrain.tankDrive(0.5, 0.5);
                if(clock.get() > 1)
                {
                    Robot.driveTrain.tankDrive(0,0);
                }
            break;

            case(4):    //straightBackwardMid(4)
                Robot.driveTrain.tankDrive(0.45, 0.45);
                if(clock.get() > 1)
                {
                    Robot.driveTrain.tankDrive(0,0);
                }
            break;

            case(5):    //tipToeBackward(5)
                Robot.driveTrain.tankDrive(0.3, 0.3);
                if(clock.get() > 1.0)
                {
                    Robot.driveTrain.tankDrive(0,0);
                }
            break;
                


            
        }


    }
}
