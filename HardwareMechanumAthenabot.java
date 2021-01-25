package org.firstinspires.ftc.teamcode;

//import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

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
   //public DcMotor  hang_motor = null; // Robot hang and landing

   //public DcMotor  hinge = null; // hinge for arm
   //public DcMotor  intake_motor = null ; //intake motor for arm
   public DcMotor intake_motor = null; // intake motor for clamp
   public DcMotor fly_wheel = null;
   public DcMotor ramp = null;
   //public DcMotor intake2_motor = null;
   //public DcMotor flip_motor = null; // slide motor for stacking
     // public DcMotor stack2_motor = null;

   public Servo push = null; // servo for foundation
   public Servo push2 = null;
   public Servo grab = null;
   public Servo grab2 = null;
   public Servo turn = null;

   public ColorSensor colorSensor = null; // Sensor for Beacon
   public LightSensor LightSensorBottom = null; // Sensor for line following
   //public OpticalDistanceSensor odsSensor = null;  // Sensor for dist measurement

   public static final double SERVO_POS1       =  1.0  ; //
   public static final double SERVO_POS0       =  0.0 ; //
   public static final double SERVO_MID       =  0.5 ; //
   public static final double SERVO_LEFT_MIN       =  SERVO_POS0 + 0.3 ;
   public static final double SERVO_LEFT_MAX       =  SERVO_POS1 ;
   public static final double SERVO_PUSH_MIN = 0.0;
   public static final double SERVO_PUSH_MAX = -2.0;
   public static final double SERVO_PUSH2_MIN = 0.0;
   public static final double SERVO_PUSH2_MAX = -2.0;
   public static final double SERVO_GRAB_MIN = 0.0;
   public static final double SERVO_GRAB_MAX = 2.0;
   public static final double SERVO_GRAB2_MIN = 0.0;
   public static final double SERVO_GRAB2_MAX = 2.0;
   public static final double SERVO_TURN_MIN = 0.0;
   public static final double SERVO_TURN_MAX = 2.0;
   //public static final double HINGE_POWER  = 0.75 ;
   //public static final double FLIP_POWER = 0.75 ;
   public static final double fly_wheel_power = 0.75;
   public static final double ramp_power = 0.75;
   //public static final double STACK2_POWER = 0.75;
   public static final double intake_motor_power = 0.25 ;
   //public static final double INTAKE2_POWER = 0.25;


   //Use MR Core Device Discovery to change address
   //I2cAddr i2CAddressColorFront = I2cAddr.create8bit(0x3c);
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
       try {

           left_front_drv_Motor = hwMap.dcMotor.get("frontLeft");
       } catch (Exception p_exeception) {


           left_front_drv_Motor = null;
       }
       //left back drive motor

       try {

           left_back_drv_Motor = hwMap.dcMotor.get("backLeft");
       } catch (Exception p_exeception) {


           left_back_drv_Motor = null;
       }
       // right front drive motor
       try {

           right_front_drv_Motor = hwMap.dcMotor.get("frontRight");
       } catch (Exception p_exeception) {

           right_front_drv_Motor = null;
       }
       //right back drive motor
       try {

           right_back_drv_Motor = hwMap.dcMotor.get("backRight");
       } catch (Exception p_exeception) {


           right_back_drv_Motor = null;
       }

       try {
           colorSensor = hwMap.colorSensor.get("color");

       } catch (Exception p_exception) {

           colorSensor = null;
       }
       //hinge motor for arm

       //try
       //{

       /*    hinge   = hwMap.dcMotor.get("hinge");
       }
       catch (Exception p_exeception)
       {


           hinge = null;
       }*/
           //stack motor for stacking skystones
       /*try {

           flip_motor = hwMap.dcMotor.get("extension");
       } catch (Exception p_exeception) {


           flip_motor = null;
       }*/

       /*try {

           stack2_motor = hwMap.dcMotor.get("stack2");
       } catch (Exception p_exeception) {


           stack2_motor = null;
       }*/


           //intake motor for clamp
       try {

           intake_motor = hwMap.dcMotor.get("arm");
       } catch (Exception p_exeception) {


           intake_motor = null;
       }
       try {
           fly_wheel = hwMap.dcMotor.get("fly_wheel");
       } catch (Exception p_exception){
           fly_wheel = null;
       }

       try {
           ramp = hwMap.dcMotor.get("ramp");
       } catch (Exception p_exception){
           ramp = null;
       }
       /*try {

           intake2_motor = hwMap.dcMotor.get("intake2");
       } catch (Exception p_exeception) {


           intake2_motor = null;
       }*/

           // Servos :


           try {
               //v_motor_left_drive = hardwareMap.dcMotor.get ("l_drv");
               //v_motor_left_drive.setDirection (DcMotor.Direction.REVERSE);
               push = hwMap.servo.get("fingers2");
               if (push != null) {
                   push.setPosition(SERVO_PUSH_MIN);
               }


           }
           catch (Exception p_exeception) {
               //m_warning_message ("l_drv");
               //   DbgLog.msg (p_exeception.getLocalizedMessage ());

               push = null;
           }

           try {
               //v_motor_left_drive = hardwareMap.dcMotor.get ("l_drv");
               //v_motor_left_drive.setDirection (DcMotor.Direction.REVERSE);
               push2 = hwMap.servo.get("fingers2");
               if (push2 != null) {
                   push2.setPosition(SERVO_PUSH2_MIN);
               }


           }
           catch (Exception p_exeception) {
           //m_warning_message ("l_drv");
           //   DbgLog.msg (p_exeception.getLocalizedMessage ());

               push2 = null;
           }

           try {
           //v_motor_left_drive = hardwareMap.dcMotor.get ("l_drv");
           //v_motor_left_drive.setDirection (DcMotor.Direction.REVERSE);
           grab = hwMap.servo.get("fingers");
           if (grab != null) {
               grab.setPosition(SERVO_GRAB_MIN);
           }


           }
           catch (Exception p_exeception) {
           //m_warning_message ("l_drv");
           //   DbgLog.msg (p_exeception.getLocalizedMessage ());

               grab = null;
           }
       try {

       grab2 = hwMap.servo.get("fingers");
       if (grab2 != null) {
           grab2.setPosition(SERVO_GRAB2_MIN);
       }


       }
           catch (Exception p_exeception) {
   //m_warning_message ("l_drv");
   //   DbgLog.msg (p_exeception.getLocalizedMessage ());

       grab2 = null;
       }

           try {
           //v_motor_left_drive = hardwareMap.dcMotor.get ("l_drv");
           //v_motor_left_drive.setDirection (DcMotor.Direction.REVERSE);
           turn = hwMap.servo.get("hand_x");
           if (turn != null) {
               turn.setPosition(SERVO_TURN_MIN);
           }


           }
           catch (Exception p_exeception) {
           //m_warning_message ("l_drv");
           //   DbgLog.msg (p_exeception.getLocalizedMessage ());

               turn = null;
           }

           //set dc motors to run without an encoder  and set intial power to 0
           //l_f_drv
           if (left_front_drv_Motor != null) {
               //l_return = left_drv_Motor.getPower ();
               left_front_drv_Motor.setDirection(DcMotor.Direction.FORWARD); // FORWARD was moving it backwards
               left_front_drv_Motor.setPower(0);
               left_front_drv_Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
           }
           //l_b_drv
           if (left_back_drv_Motor != null) {
               //l_return = left_drv_Motor.getPower ();
               left_back_drv_Motor.setDirection(DcMotor.Direction.FORWARD); // FORWARD was moving it backwards
               left_back_drv_Motor.setPower(0);
               left_back_drv_Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
           }

           //r_f_drv
           if (right_front_drv_Motor != null) {
               //l_return = left_drv_Motor.getPower ();
               right_front_drv_Motor.setDirection(DcMotor.Direction.REVERSE);// REVERSE was moving it backwards
               right_front_drv_Motor.setPower(0);
               right_front_drv_Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
           }
           //r_b_drv
           if (right_back_drv_Motor != null) {
               //l_return = left_drv_Motor.getPower ();
               right_back_drv_Motor.setDirection(DcMotor.Direction.REVERSE);// REVERSE was moving it backwards
               right_back_drv_Motor.setPower(0);
               right_back_drv_Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
           }
           //hinge
       /*if (hinge != null)
       {
           //l_return = left_drv_Motor.getPower ();
           hinge.setPower(0);
           hinge.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       }*/
           //hang
           /*if (flip_motor != null) {
               flip_motor.setPower(0);
               flip_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
           }*/
           /*if (stack2_motor != null) {
               stack2_motor.setPower(0);
               stack2_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
           }*/
           //intake
           if (intake_motor != null) {
               intake_motor.setDirection(DcMotor.Direction.FORWARD);
               intake_motor.setPower(0);
               intake_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
           }
           if (fly_wheel != null) {
               fly_wheel.setDirection(DcMotor.Direction.FORWARD);
               fly_wheel.setPower(0);
               fly_wheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
           }

           if (ramp != null){
               ramp.setDirection(DcMotor.Direction.FORWARD);
               ramp.setPower(0);
               ramp.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
           }
           /*if (intake2_motor != null) {
               intake2_motor.setDirection(DcMotor.Direction.REVERSE);
               intake2_motor.setPower(0);
               intake2_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
           }*/

       /*
       initalize the colorSensor and colorSensor
       */
           try {
               colorSensor = hwMap.colorSensor.get("color");
           } catch (Exception p_exeception) {
               //m_warning_message ("colr_f");
               //   DbgLog.msg (p_exeception.getLocalizedMessage ());
               colorSensor = null;
           }


           try {
               LightSensorBottom = hwMap.lightSensor.get("ods");
           } catch (Exception p_exeception) {
               //m_warning_message ("ods");
               //   DbgLog.msg (p_exeception.getLocalizedMessage ());
               LightSensorBottom = null;
           }


           if (colorSensor != null) {
               //ColorFront reads beacon light and is in passive mode
               //colorSensor.setI2cAddress(i2CAddressColorFront);
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
   public double scaleInput( double dVal ) {
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
   public double a_left_front_drive_power()
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
   /*double a_hinge_power ()
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
   /*double a_flip_power ()
   {
       double l_return = 0.0;

       if (flip_motor != null) {
           l_return = flip_motor.getPower();
       }

       return l_return;
   }*/

   /*double a_stack2_power ()
   {
       double l_return = 0.0;

       if (stack2_motor != null) {
           l_return = stack2_motor.getPower();
       }

       return l_return;
   }*/


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

   double a_fly_wheel_power ()
   {
       double l_return = 0.0;
       if (fly_wheel != null){
           l_return = fly_wheel.getPower();
       }
       return l_return;
   }

   double a_ramp_power ()
   {
       double l_return =0.0;
       if (ramp != null){
           l_return = ramp.getPower();
       }
       return l_return;
   }

   /*double a_intake2_power ()
   {
       double l_return = 0.0;

       if (intake2_motor != null) {
           l_return = intake2_motor.getPower();
       }

       return l_return;
   }*/


   // marker - servo position
   //
   /**
    * Access the marker servo position.
    */
   double a_push_position ()
   {
       double l_return = 0.0;

       if (push != null)
       {
           l_return = push.getPosition ();
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
   void set_push_position (double p_position) {
       //
       // Ensure the specific value is legal.
       //
       double l_position = Range.clip
               (p_position , SERVO_PUSH_MIN
                       , SERVO_PUSH_MAX); // Servo position is restricted to protect from mechanical damage



       //
       // Set the right  value.
       //
       if (push!= null)
       {
           push.setPosition (l_position);
       }

   }

   double a_push2_position ()
   {
       double l_return = 0.0;

       if (push2 != null)
       {
           l_return = push2.getPosition ();
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
   void set_push2_position (double p_position) {
       //
       // Ensure the specific value is legal.
       //
       double l_position = Range.clip
               (p_position , SERVO_PUSH2_MIN
                       , SERVO_PUSH2_MAX); // Servo position is restricted to protect from mechanical damage



       //
       // Set the right  value.
       //
       if (push2!= null)
       {
           push2.setPosition (l_position);
       }

   }

   double a_grab_position ()
   {
       double l_return = 0.0;

       if (grab != null)
       {
           l_return = grab.getPosition ();
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
   void set_grab_position (double p_position) {
       //
       // Ensure the specific value is legal.
       //
       double l_position = Range.clip
               (p_position , SERVO_GRAB_MIN
                       , SERVO_GRAB_MAX); // Servo position is restricted to protect from mechanical damage



       //
       // Set the right  value.
       //
       if (grab!= null)
       {
           grab.setPosition (l_position);
       }

   }

   double a_grab2_position ()
   {
       double l_return = 0.0;

       if (grab2 != null)
       {
           l_return = grab2.getPosition ();
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
   void set_grab2_position (double p_position) {
       //
       // Ensure the specific value is legal.
       //
       double l_position = Range.clip
               (p_position , SERVO_GRAB2_MIN
                       , SERVO_GRAB2_MAX); // Servo position is restricted to protect from mechanical damage



       //
       // Set the right  value.
       //
       if (grab2!= null)
       {
           grab2.setPosition (l_position);
       }

   }

   double a_turn_position ()
   {
       double l_return = 0.0;

       if (turn != null)
       {
           l_return = turn.getPosition ();
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
   void set_turn_position (double p_position) {
       //
       // Ensure the specific value is legal.
       //
       double l_position = Range.clip
               (p_position , SERVO_TURN_MIN
                       , SERVO_TURN_MAX); // Servo position is restricted to protect from mechanical damage



       //
       // Set the right  value.
       //
       if (turn!= null)
       {
           turn.setPosition (l_position);
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









