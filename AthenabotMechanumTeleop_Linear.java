/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

/**
* This OpMode uses the common Pushbot hardware class to define the devices on the robot.
* All device access is managed through the HardwarePushbot class.
* The code is structured as a LinearOpMode
*
* This particular OpMode executes a POV Game style Teleop for a PushBot
* In this mode the Gamepad1 left stick moves the robot FWD and back, the Gamepad1 Right stick turns left and right.
* Gamepad2 left stick will control the ball collector motors to suck in
* the ball or shoot them. Gamepad2 left stick will also control the horizontal roller to move the balls in.
* or out.
*
* Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
* Remove or comment out thae @Disabled line to add this opmode to the Driver Station OpMode list
*/

@TeleOp(name="AthenabotMechanumTeleop_Linear: Teleop", group="Athenabot")
//@Disabled
public class AthenabotMechanumTeleop_Linear extends LinearOpMode {

   /* Declare OpMode members. */
   HardwareMechanumAthenabot robot  = new HardwareMechanumAthenabot();

   // Use a Pushbot's hardware
   // could also use HardwarePushbotMatrix class.

   final double    MARKER_SPEED     = 0.05;
   final double    MAX_SPEED      = 1.0;

   @Override
   public void runOpMode() throws InterruptedException {
       double left;
       double right;
       double max;
       double v1 = 0.0;
       double v2 = 0.0;
       double v3 = 0.0;
       double v4 = 0.0;

       /* Initialize the hardware variables.
        * The init() method of the hardware class does all the work here
        */
       robot.init(hardwareMap);

       // Send telemetry message to signify robot waiting;
       telemetry.addData("Say", "Hello Driver");    //
       telemetry.update();

       // Wait for the game to start (driver presses PLAY)
       waitForStart();
       //robot.leftlift.setPosition(robot.SERVO_POS0);
       //robot.rightlift.setPosition(0.9);


       // run until the end of the match (driver presses STOP)
       while (opModeIsActive()) {


           //holonomic(Speed, Turn, Strafe, MAX_SPEED );

           //      Left Front = +Speed + Turn - Strafe      Right Front = +Speed - Turn + Strafe
           //      Left Rear  = +Speed + Turn + Strafe      Right Rear  = +Speed - Turn - Strafe

           //double Speed = -gamepad1.left_stick_y;
           //double Turn = gamepad1.left_stick_x;
           double Speed = -gamepad1.left_stick_x;
           double Turn = gamepad1.left_stick_y;
           double Strafe = -gamepad1.right_stick_x;

           double Magnitude = Math.abs(Speed) + Math.abs(Turn) + Math.abs(Strafe);
           Magnitude = (Magnitude > 1) ? Magnitude : 1; //Set scaling to keep -1,+1 range
           // Wheels on joystick - gamepad 1

           if (robot.left_front_drv_Motor != null) {
               robot.left_front_drv_Motor.setPower(
                       Range.scale((robot.scaleInput(Speed) +
                                       robot.scaleInput(Turn) -
                                       robot.scaleInput(Strafe)),
                               -Magnitude, +Magnitude, -MAX_SPEED, +MAX_SPEED));
           }
           if (robot.left_back_drv_Motor != null) {
               robot.left_back_drv_Motor.setPower(
                       Range.scale((robot.scaleInput(Speed) +
                                       robot.scaleInput(Turn) +
                                       robot.scaleInput(Strafe)),
                               -Magnitude, +Magnitude, -MAX_SPEED, +MAX_SPEED));
           }
           if (robot.right_front_drv_Motor != null) {
               robot.right_front_drv_Motor.setPower(
                       Range.scale((robot.scaleInput(Speed) -
                                       robot.scaleInput(Turn) -
                                       robot.scaleInput(Strafe)),
                               -Magnitude, +Magnitude, -MAX_SPEED, +MAX_SPEED));
           }
           if (robot.right_back_drv_Motor != null) {
               robot.right_back_drv_Motor.setPower(
                       Range.scale((robot.scaleInput(Speed) -
                                       robot.scaleInput(Turn) +
                                       robot.scaleInput(Strafe)),
                               -Magnitude, +Magnitude, -MAX_SPEED, +MAX_SPEED));
           }

          // Hinge on a and x - gamepad 2

           if (robot.hinge != null) {
               if (gamepad2.a) { // cap ball up
                   robot.hinge.setDirection(DcMotor.Direction.FORWARD); //might need to change to REVERSE
                   robot.hinge.setPower(robot.HINGE_POWER);
               } else if (gamepad2.x) {  // cap ball
                   robot.hinge.setDirection(DcMotor.Direction.REVERSE); //might need to change to FORWARD
                   robot.hinge.setPower(robot.HINGE_POWER);
               } else {
                   robot.hinge.setPower(0.0);
               }
           }
           if (robot.hang_motor != null) {
               if (gamepad2.left_bumper) { // ball shoot
                   robot.hang_motor.setDirection(DcMotor.Direction.FORWARD); //might need to change to FORWARD/REVERSE
                   robot.hang_motor.setPower(robot.HANG_POWER);
               } else if (gamepad2.left_trigger > 0) {
                   robot.hang_motor.setDirection(DcMotor.Direction.REVERSE);
                   robot.hang_motor.setPower(robot.HANG_POWER);
               } else {
                   robot.hang_motor.setPower(0.0);

               }

           }
           // hangigng mechanism - gamepad 2 - trigger and bumper
           if (robot.intake_motor != null) {
               if (gamepad2.right_bumper) { // ball shoot
                   robot.intake_motor.setDirection(DcMotor.Direction.FORWARD); //might need to change to FORWARD/REVERSE
                   robot.intake_motor.setPower(robot.INTAKE_POWER);
               } else if (gamepad2.right_trigger > 0) {
                   robot.intake_motor.setDirection(DcMotor.Direction.REVERSE);
                   robot.intake_motor.setPower(robot.INTAKE_POWER);
               } else {
                   robot.intake_motor.setPower(0.0);

               }

           }



           if (robot.marker != null) {
               if (gamepad2.y) {
                   robot.set_marker_position((robot.a_marker_position() - MARKER_SPEED));
               } else if (gamepad2.b) {
                   robot.set_marker_position(robot.a_marker_position() + MARKER_SPEED);
               }
           }


       }
       telemetry.addData
               ( "01"
                       , "Left Drive Power: "
                               + robot.a_left_front_drive_power()
               );
       telemetry.addData
               ( "02"
                       , "Right Drive Power: "
                               + robot.a_right_front_drive_power()
               );
       telemetry.addData
               ( "03"
                       , "Left Drive Power: "
                               + robot.a_left_back_drive_power()
               );
       telemetry.addData
               ( "04"
                       , "Right Drive Power: "
                               + robot.a_right_back_drive_power()
               );

       telemetry.addData
               ( "10"
                       , "Relic Claw Position: "
                               + robot.a_marker_position()
               );


       telemetry.addData
               ( "12"
                       , "Hinge Power: "
                               + robot.a_hinge_power()
               );



       telemetry.update();

       // Pause for metronome tick.  40 mS each cycle = update 25 times a second.
       robot.waitForTick(40);
   }
}


