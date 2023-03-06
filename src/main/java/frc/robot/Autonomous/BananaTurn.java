package frc.robot.Autonomous;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Mechanisms.BananaDriveTrain;
import frc.robot.Robot.BananaConstants;
import frc.robot.Robot.Robot;

public class BananaTurn {
    
    /***************************************************************************
	 * PID Turn Constants
	 ***************************************************************************/
    static private double PID_TURN_THRESHOLD   = 0.5;

	/***************************************************************************
	 * PID_TURN_DELTAERROR_THRESHOLD_HI - Delta Error Values larger than this are
	 * considered invalid and will be ignored PID_TURN_DELTAERROR_THRESHOLD_LO -
	 * When drivetrain power drops below the PID_TURN_MIN_xxx_POWER, we will check
	 * to see if deltaError is below this threshold before setting power at
	 * PID_TURN_MIN_xxx_POWER.
	 ***************************************************************************/
	final static private double PID_TURN_DELTAERROR_THRESHOLD_HI = 4.0;
	final private static double PID_TURN_DELTAERROR_THRESHOLD_LO = 0.11;

	final static private double PID_TURN_FILTER_CONSTANT    = 0.7;
    final static private double PID_TURN_POWER_SCALE_FACTOR = 1.0;

	final static private double PID_TURN_KP = 0.08;
    final static private double PID_TURN_KI = 0.0;
    final static private double PID_TURN_KD = 0.012; // 0.0744

	final static private double TURN_MAX_POWER =  1.0;
	final static private double TURN_MIN_POWER = -1.0;

	final static private double PID_TURN_MIN_POS_POWER = 0.6; // 0.4 is min power to move robot when it is stopped
	final static private double PID_TURN_MIN_NEG_POWER = -PID_TURN_MIN_POS_POWER;


	
	/***************************************************************************
	 * PID Turn Variables
	 ***************************************************************************/
	private static Timer functionTimer;
	private static Timer pdTimer;

    private static double pidTurnkP = PID_TURN_KP;
    private static double pidTurnkI = PID_TURN_KI;
    private static double pidTurnkD = PID_TURN_KD;

    private static double previousError;
	private static double currentError; 
    private static double totalError;
	private static double deltaError;
	private static double derivative;
	private static double deltaT;

	private static double power;

	private static double currentAngle;
	private static double targetAngle;

	private static double timeout;
	private static double loopDelay = 0.015;

	private static double previousDerivative = 0;
    

    public static void turnPID(double degreesToTurn, double timeoutSeconds){

        Robot.navx.reset();
		Timer.delay(BananaConstants.NAVX_RESET_WAIT_TIME);

		pdTimer       = new Timer();
		functionTimer = new Timer();
		functionTimer.reset();
		functionTimer.start();

		boolean done      = false;
		boolean firstTime = true;

		setPidValues(degreesToTurn);

		previousError = 0.0;
		totalError    = 0.0;

		currentAngle  = Robot.navx.getAngle();
		targetAngle   = degreesToTurn + currentAngle;
		currentError  = targetAngle   - currentAngle;

		//targetAngleAbs = Math.abs(targetAngle);
        pdTimer.reset();
		pdTimer.start();

		while (done == false) {
			currentAngle = Robot.navx.getAngle();

			deltaT = pdTimer.get();
			pdTimer.stop();
			pdTimer.reset();
			pdTimer.start();

			//currentAngleAbs = Math.abs(currentAngle);


			// calculates proportional term
			currentError = targetAngle - currentAngle;

			if (Math.abs(currentError) < PID_TURN_THRESHOLD) //if within half a degree, plus or minus
            {
				done = true;
			} 
            else 
            {
				if (functionTimer.get() > timeoutSeconds) 
                {
					done = true;
				} 
                else 
                {
					
					deltaError = currentError - previousError;

					if (firstTime == false) 
                    {

						
						if ((deltaError == 0.0) && (Math.abs(currentError) > 3.0)) 
                        {
							derivative = previousDerivative;
						} 
                        else 
                        {

							if (Math.abs(deltaError) > PID_TURN_DELTAERROR_THRESHOLD_HI) 
                            {
								derivative = previousDerivative;
							} 
                            else 
                            {

								/**********************************************************
								 * We have a good deltaError value. Filter the derivative value to smooth out
								 * jumps in derivative value
								 **********************************************************/
                                
								derivative = PID_TURN_FILTER_CONSTANT * previousDerivative
										+ ((1 - PID_TURN_FILTER_CONSTANT) * (deltaError / deltaT));
							}
						}
					} 
                    else 
                    {
						firstTime = false;
						derivative = 0;
					}

					// Save values for next iteration
					previousDerivative = derivative;
					previousError      = currentError;
                    

                    power = PID_TURN_POWER_SCALE_FACTOR
                            * ((pidTurnkP * currentError) + (PID_TURN_KI * totalError) + (PID_TURN_KD * derivative));

					// Verify we have not exceeded max power when turning right or left
					if (power > TURN_MAX_POWER)
						power = TURN_MAX_POWER;

					if (power < TURN_MIN_POWER)
						power = TURN_MIN_POWER;

					/**********************************************************************
					 * We need to make sure drivetrain power doesn't get too low but we also need to
					 * allow the robot to gradually brake. The brake condition is defined as when
					 * deltaError is > PID_TURN_DELTAERROR_THRESHOLD_LO If deltaError is <
					 * PID_TURN_DELTAERROR_THRESHOLD_LO, then we will set power to
					 * PID_TURN_MIN_xxx_POWER.
					 **********************************************************************/
					if (power >= 0.0) {
						if (power < PID_TURN_MIN_POS_POWER && Math.abs(deltaError) < PID_TURN_DELTAERROR_THRESHOLD_LO)
							power = PID_TURN_MIN_POS_POWER;
					} else if (power < 0.0) {
						if (power > PID_TURN_MIN_NEG_POWER && Math.abs(deltaError) < PID_TURN_DELTAERROR_THRESHOLD_LO)
							power = PID_TURN_MIN_NEG_POWER;
					}

					//Power will be positive if turning right and negative if turning left
					 
					Robot.driveTrain.tankDrive(power*0.7, power); //check logic
                    Timer.delay(loopDelay);
                }
            }
        }
        currentAngle = Robot.navx.getAngle();

        Robot.driveTrain.tankDrive(0.0, 0.0); // makes robot stop
	    currentAngle = Robot.navx.getAngle();
                
        functionTimer.stop();
		pdTimer.stop();


        
    }

    public static void setPidValues(double degreesToTurn) 
        {
            double degreesToTurnAbs;
    
            degreesToTurnAbs = Math.abs(degreesToTurn);
    
            if (degreesToTurnAbs <= 25.0) {
                pidTurnkP = 0.090;
                pidTurnkD = 0.024;
            }
            if (degreesToTurnAbs <= 30.0) {
                pidTurnkP = 0.110;   //0.126 at 12.0 V;
                pidTurnkD = 0.026;
                PID_TURN_THRESHOLD = 0.75;
                loopDelay = 0.007;
            }
            else if (degreesToTurnAbs <= 35.0) {
                pidTurnkP = 0.090;
                pidTurnkD = 0.020;
            }
            else if (degreesToTurnAbs <= 40.0) {
                pidTurnkP = 0.086;
                pidTurnkD = 0.024;
            }
            else if (degreesToTurnAbs <= 45.0) {
                pidTurnkP = 0.090;
                pidTurnkD = 0.030;
                PID_TURN_THRESHOLD = 0.75;
                loopDelay = 0.007;
            }
            else if (degreesToTurnAbs <= 50.0) {
                pidTurnkP = 0.100;
                pidTurnkD = 0.028;
                PID_TURN_THRESHOLD = 0.75;
                loopDelay = 0.007;
            }
            else { //if degreesToTurnAbs > 50.0
                pidTurnkP = 0.080;  //PID_TURN_KP;
                pidTurnkD = 0.030;  //PID_TURN_KD;
                PID_TURN_THRESHOLD = 0.5;
                loopDelay = 0.010;
            }
    
        }   //End of setPidValues()

}

    

