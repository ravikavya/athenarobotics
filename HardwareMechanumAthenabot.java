package org.firstinspires.ftc.teamcode;

//import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

//import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

/**
* This is NOT an opmode.
*
* This class can be used to define all the specific hardware for a single robot.
* In this case that robot is a Pushbot.
* See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
*
* This hardware class assumes the following device names have been configured on the robot:
* Note:  All names are lower case and some have single spaces between words.
*
* Motor channel:  Left  drive motor:        "left_drive"
* Motor channel:  Right drive motor:        "right_drive"
* Motor channel:  Manipulator drive motor:  "roller_drive"
* Servo channel:  Servo to open left claw_left:  "left_hand"
* Servo channel:  Servo to open right claw_left: "right_hand"
*/
public class HardwareMechanumAthenabot
{
   /* Public OpMode members. */

   public DcMotor  left_front_drv_Motor   = null;    // Robot left front drive
   public DcMotor  right_front_drv_Motor  = null;    // Robot right front drive
   public DcMotor  left_back_drv_Motor   = null;    // Robot left back drive
   public DcMotor  right_back_drv_Motor  = null;    // Robot right back drive
   public DcMotor  hang_motor = null; // Robot hang and landing
   public DcMotor  hinge = null; // hinge for arm
   public DcMotor  intake_motor = null ; //intake motor for arm



   public Servo marker = null ; // servo for marker


   public ColorSensor colorSensor = null; // Sensor for Beacon
   public LightSensor LightSensorBottom = null; // Sensor for line following
   //public OpticalDistanceSensor odsSensor = null;  // Sensor for dist measurement

   public static final double SERVO_POS1       =  1.0  ; //
   public static final double SERVO_POS0       =  0.0 ; //
   public static final double SERVO_MID       =  0.5 ; //
   public static final double SERVO_LEFT_MIN       =  SERVO_POS0 + 0.3 ;
   public static final double SERVO_LEFT_MAX       =  SERVO_POS1 ;
   public static final double SERVO_MARKER_MIN = 0.0;
   public static final double SERVO_MARKER_MAX = 1.0;
   public static final double HINGE_POWER  = 0.75 ;
   public static final double HANG_POWER = 0.75 ;
   public static final double INTAKE_POWER = 0.25 ;


   //Use MR Core Device Discovery to change address
   I2cAddr i2CAddressColorFront = I2cAddr.create8bit(0x3c);
   //I2cAddr i2CAddressColorBottom = I2cAddr.create8bit(0x4c);


   /* local OpMode members. */
   HardwareMap hwMap           =  null;
   private ElapsedTime period  = new ElapsedTime();

   /* Constructor */
   public HardwareMechanumAthenabot(){

   }

   /* Initialize standard Hardware interfaces */
   public void init(HardwareMap ahwMap) {
       // Save reference to Hardware map
       hwMap = ahwMap;
      // left frond drive motor
       try
       {

           left_front_drv_Motor   = hwMap.dcMotor.get("l_f_drv");
       }
       catch (Exception p_exeception)
       {


           left_front_drv_Motor = null;
       }
       //left back drive motor

       try
       {

           left_back_drv_Motor   = hwMap.dcMotor.get("l_b_drv");
       }
       catch (Exception p_exeception)
       {


           left_back_drv_Motor = null;
       }
       // right front drive motor
       try
       {

           right_front_drv_Motor   = hwMap.dcMotor.get("r_f_drv");
       }
       catch (Exception p_exeception)
       {

           right_front_drv_Motor = null;
       }
       //right back drive motor
       try
       {

           right_back_drv_Motor   = hwMap.dcMotor.get("r_b_drv");
       }
       catch (Exception p_exeception)
       {


           right_back_drv_Motor = null;
       }

    //hinge motor for arm

       try
       {

           hinge   = hwMap.dcMotor.get("hinge");
       }
       catch (Exception p_exeception)
       {


           hinge = null;
       }
       //hang motor to land
       try
       {

           hang_motor  = hwMap.dcMotor.get("hang");
       }
       catch (Exception p_exeception)
       {


          hang_motor= null;
       }
       //intake motor
       try
       {

           intake_motor  = hwMap.dcMotor.get("intake");
       }
       catch (Exception p_exeception)
       {


           hang_motor= null;
       }

       // Servos :


       try
       {
           //v_motor_left_drive = hardwareMap.dcMotor.get ("l_drv");
           //v_motor_left_drive.setDirection (DcMotor.Direction.REVERSE);
           marker  = hwMap.servo.get("marker");
           if (marker != null) {
               marker.setPosition(SERVO_LEFT_MIN);
           }


       }

       catch (Exception p_exeception)
       {
           //m_warning_message ("l_drv");
           //   DbgLog.msg (p_exeception.getLocalizedMessage ());

           marker = null;
       }


       //set dc motors to run without an encoder  and set intial power to 0
       //l_f_drv
       if (left_front_drv_Motor != null)
       {
           //l_return = left_drv_Motor.getPower ();
           left_front_drv_Motor.setDirection(DcMotor.Direction.FORWARD); // FORWARD was moving it backwards
           left_front_drv_Motor.setPower(0);
           left_front_drv_Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       }
       //l_b_drv
       if (left_back_drv_Motor != null)
       {
           //l_return = left_drv_Motor.getPower ();
           left_back_drv_Motor.setDirection(DcMotor.Direction.FORWARD); // FORWARD was moving it backwards
           left_back_drv_Motor.setPower(0);
           left_back_drv_Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       }

       //r_f_drv
       if (right_front_drv_Motor != null)
       {
           //l_return = left_drv_Motor.getPower ();
           right_front_drv_Motor.setDirection(DcMotor.Direction.FORWARD);// REVERSE was moving it backwards
           right_front_drv_Motor.setPower(0);
           right_front_drv_Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       }
       //r_b_drv
       if (right_back_drv_Motor != null)
       {
           //l_return = left_drv_Motor.getPower ();
           right_back_drv_Motor.setDirection(DcMotor.Direction.FORWARD);// REVERSE was moving it backwards
           right_back_drv_Motor.setPower(0);
           right_back_drv_Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       }
       //hinge
       if (hinge != null)
       {
           //l_return = left_drv_Motor.getPower ();
           hinge.setPower(0);
           hinge.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       }
       //hang
       if(hang_motor != null)

       {
           hang_motor.setPower(0);
           hang_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       }
      //intake
       if(intake_motor != null)

       {
           intake_motor.setPower(0);
           intake_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       }

/*
initalize the colorSensor and colorSensor
*/
       try {
           colorSensor = hwMap.colorSensor.get("color");
       }
       catch (Exception p_exeception)
       {
           //m_warning_message ("colr_f");
           //   DbgLog.msg (p_exeception.getLocalizedMessage ());
           colorSensor = null;
       }


       try {
           LightSensorBottom = hwMap.lightSensor.get("ods");
       }
       catch (Exception p_exeception)
       {
           //m_warning_message ("ods");
           //   DbgLog.msg (p_exeception.getLocalizedMessage ());
           LightSensorBottom = null;
       }


       if (colorSensor != null) {
           //ColorFront reads beacon light and is in passive mode
           colorSensor.setI2cAddress(i2CAddressColorFront);
           colorSensor.enableLed(false);
       }

       if (LightSensorBottom != null) {
           //OpticalDistance sensor measures dist from the beacon
           LightSensorBottom.enableLed(false);
       }

   }

   /*
    * This method scales the joystick input so for low joystick values, the
    * scaled value is less than linear. This is to make it easier to drive
    * the robot more precisely at slower speeds.
    */
   double scaleInput(double dVal) {
       double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
               0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

       // get the corresponding index for the scaleInput array.
       int index = (int) (dVal * 16.0);
       if (index < 0) {
           index = -index;
       } else if (index > 16) {
           index = 16;
       }

       double dScale = 0.0;
       if (dVal < 0) {
           dScale = -scaleArray[index];
       } else {
           dScale = scaleArray[index];
       }

       return dScale;
   }


   /**
    * Access the left front drive motor's power level.
    */
   double a_left_front_drive_power()
   {
       double l_return = 0.0;
       if (left_front_drv_Motor != null)
       {
           l_return = left_front_drv_Motor.getPower ();
       }

       return l_return;

   } // a_left_drive_power

   /**
    * Access the left back drive motor's power level.
    */
   double a_left_back_drive_power()
   {
       double l_return = 0.0;
       if (left_back_drv_Motor != null)
       {
           l_return = left_back_drv_Motor.getPower ();
       }

       return l_return;

   } // a_left_drive_power



   /**
    * Access the right front drive motor's power level.
    */
   double a_right_front_drive_power ()
   {
       double l_return = 0.0;

       if (right_front_drv_Motor != null)
       {
           l_return = right_front_drv_Motor.getPower ();
       }

       return l_return;

   } // a_right_front_drive_power

   /**
    * Access the right back drive motor's power level.
    */
   double a_right_back_drive_power ()
   {
       double l_return = 0.0;

       if (right_back_drv_Motor != null)
       {
           l_return = right_back_drv_Motor.getPower ();
       }

       return l_return;

   } // a_right_drive_power

   /**
    * Access the Hinge motor's power level.
    */
   double a_hinge_power ()
   {
       double l_return = 0.0;

       if (hinge != null) {
           l_return = hinge.getPower();
       }

       return l_return;
   }
   /**
    * Access the hang motor's power level.
    */
     double a_hang_power ()
         {
             double l_return = 0.0;

             if (hang_motor != null) {
                 l_return = hang_motor.getPower();
            }

             return l_return;
         }


   /**
    * Access the intake motor's power level.
    */
   //
   double a_intake_power ()
   {
       double l_return = 0.0;

       if (intake_motor != null) {
           l_return = intake_motor.getPower();
       }

       return l_return;
   }


   // marker - servo position
   //
   /**
    * Access the marker servo position.
    */
   double a_marker_position ()
   {
       double l_return = 0.0;

       if (marker != null)
       {
           l_return = marker.getPosition ();
       }

       return l_return;

   }
   //
   //
   // set_marker_position
   //
   /**
    * Change the marker servo position.
    */
   void set_marker_position (double p_position) {
       //
       // Ensure the specific value is legal.
       //
       double l_position = Range.clip
               (p_position , SERVO_MARKER_MIN
                       , SERVO_MARKER_MAX); // Servo position is restricted to protect from mechanical damage



       //
       // Set the right  value.
       //
       if (marker!= null)
       {
          marker.setPosition (l_position);
       }

   }
   /***
    *
    * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
    * periodic tick.  This is used to compensate for varying processing times for each cycle.
    * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
    *
    * @param periodMs  Length of wait cycle in mSec.
    * @throws InterruptedException
    */
   public void waitForTick(long periodMs) throws InterruptedException {

       long  remaining = periodMs - (long)period.milliseconds();

       // sleep for the remaining portion of the regular cycle period.
       if (remaining > 0)
           Thread.sleep(remaining);

       // Reset the cycle clock for the next pass.
       period.reset();
   }
}







