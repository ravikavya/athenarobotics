package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Pushbot: Auto Drive By Encoder", group="Pushbot")
//@Disabled
public class autonomousWithEncodersRight extends LinearOpMode {

        /* Declare OpMode members. */
        HardwareMechanumAthenabot robot = new HardwareMechanumAthenabot();   // Use a Pushbot's hardware
        private ElapsedTime runtime = new ElapsedTime();

        static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
        static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
        static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
        static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                (WHEEL_DIAMETER_INCHES * 3.1415);
        static final double DRIVE_SPEED = 1.0;
        static final double TURN_SPEED = 0.5;

        @Override
        public void runOpMode() {

            /*
             * Initialize the drive system variables.
             * The init() method of the hardware class does all the work here
             */
            robot.init(hardwareMap);

            // Send telemetry message to signify robot waiting;
            telemetry.addData("Status", "Resetting Encoders");    //
            telemetry.update();

            robot.left_front_drv_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.right_front_drv_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.left_back_drv_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.right_back_drv_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            robot.left_front_drv_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.right_front_drv_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.left_back_drv_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.right_back_drv_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //robot.hang_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // Send telemetry message to indicate successful Encoder reset
            telemetry.addData("Path0", "Starting at %7d :%7d",
                    robot.left_front_drv_Motor.getCurrentPosition(),
                    robot.right_front_drv_Motor.getCurrentPosition(),
                    robot.left_back_drv_Motor.getCurrentPosition(),
                    robot.right_back_drv_Motor.getCurrentPosition()
            );
            telemetry.update();

            // Wait for the game to start (driver presses PLAY)
            waitForStart();

            // Step through each leg of the path,
            // Note: Reverse movement is obtained by setting a negative distance (not speed)
            //come down
            // Step 3:  Drive Backwards for 1 Second
            //robot.hang_motor.setPower(-.5);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 3.0)) {
                telemetry.addData("Path", "Leg 3: %2.5f S Elapsed", runtime.seconds());
                telemetry.update();
            }
            encoderDrive(DRIVE_SPEED, 10, 10,10);
            strafe(DRIVE_SPEED, 10,-10,-10,10,10);
            intake(DRIVE_SPEED, 10, 10);
            launch(DRIVE_SPEED, 10, -10, 10);



            telemetry.addData("Path", "Complete");
            telemetry.update();
        }

        /*
         *  Method to perfmorm a relative move, based on encoder counts.
         *  Encoders are not reset as the move is based on the current position.
         *  Move will stop if any of three conditions occur:
         *  1) Move gets to the desired position
         *  2) Move runs out of time
         *  3) Driver stops the opmode running.
         */
        public void encoderDrive(double speed,
                                 double leftInches, double rightInches,
                                 double timeoutS) {
            int newLeftTarget;
            int newRightTarget;

            // Ensure that the opmode is still active
            if (opModeIsActive()) {

                // Determine new target position, and pass to motor controller
                newLeftTarget = robot.left_front_drv_Motor.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
                newRightTarget = robot.right_front_drv_Motor.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
                newLeftTarget = robot.left_back_drv_Motor.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
                newRightTarget = robot.right_back_drv_Motor.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
                robot.left_front_drv_Motor.setTargetPosition(newLeftTarget);
                robot.right_front_drv_Motor.setTargetPosition(newRightTarget);
                robot.left_back_drv_Motor.setTargetPosition(newLeftTarget);
                robot.right_back_drv_Motor.setTargetPosition(newRightTarget);

                // Turn On RUN_TO_POSITION
                robot.left_front_drv_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.right_front_drv_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.left_back_drv_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.right_back_drv_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // reset the timeout time and start motion.
                runtime.reset();
                robot.left_front_drv_Motor.setPower(Math.abs(speed));
                robot.right_front_drv_Motor.setPower(Math.abs(speed));
                robot.left_back_drv_Motor.setPower(Math.abs(speed));
                robot.right_back_drv_Motor.setPower(Math.abs(speed));

                // keep looping while we are still active, and there is time left, and both motors are running.
                // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
                // its target position, the motion will stop.  This is "safer" in the event that the robot will
                // always end the motion as soon as possible.
                // However, if you require that BOTH motors have finished their moves before the robot continues
                // onto the next step, use (isBusy() || isBusy()) in the loop test.
                while (opModeIsActive() &&
                        (runtime.seconds() < timeoutS) &&
                        (robot.left_front_drv_Motor.isBusy() && robot.right_front_drv_Motor.isBusy() && robot.right_back_drv_Motor.isBusy() && robot.left_back_drv_Motor.isBusy())) {

                    // Display it for the driver.
                    telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                    telemetry.addData("Path2", "Running at %7d :%7d",
                            robot.left_front_drv_Motor.getCurrentPosition(),
                            robot.right_front_drv_Motor.getCurrentPosition());
                    telemetry.update();
                }


                // Stop all motion;
                robot.left_front_drv_Motor.setPower(0);
                robot.right_front_drv_Motor.setPower(0);
                robot.left_back_drv_Motor.setPower(0);
                robot.right_back_drv_Motor.setPower(0);

                // Turn off RUN_TO_POSITION
                robot.left_front_drv_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.right_front_drv_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.left_back_drv_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.right_back_drv_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                //  sleep(250);   // optional pause after each move
            }
        }

        public void strafe(double speed_strafe, double strafe_leftfrontInches, double strafe_rightfrontInches, double strafe_leftbackInches, double strafe_rightbackInches,
                           double strafe_timeoutS) {
            int newLeftFrontTarget;
            int newRightFrontTarget;
            int newLeftBackTarget;
            int newRightBackTarget;

            // Ensure that the opmode is still active
            if (opModeIsActive()) {

                // Determine new target position, and pass to motor controller
                newLeftFrontTarget = robot.left_front_drv_Motor.getCurrentPosition() + (int) (strafe_leftfrontInches * COUNTS_PER_INCH);
                newRightFrontTarget = robot.right_front_drv_Motor.getCurrentPosition() + (int) (strafe_rightfrontInches * COUNTS_PER_INCH);
                newLeftBackTarget = robot.left_back_drv_Motor.getCurrentPosition() + (int) (strafe_leftbackInches * COUNTS_PER_INCH);
                newRightBackTarget = robot.right_back_drv_Motor.getCurrentPosition() + (int) (strafe_rightbackInches * COUNTS_PER_INCH);
                robot.left_front_drv_Motor.setTargetPosition(newLeftFrontTarget);
                robot.right_front_drv_Motor.setTargetPosition(newRightFrontTarget);
                robot.left_back_drv_Motor.setTargetPosition(newLeftBackTarget);
                robot.right_back_drv_Motor.setTargetPosition(newRightBackTarget);

                // Turn On RUN_TO_POSITION
                robot.left_front_drv_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.right_front_drv_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.left_back_drv_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.right_back_drv_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // reset the timeout time and start motion.
                runtime.reset();
                robot.left_front_drv_Motor.setPower(Math.abs(speed_strafe));
                robot.right_front_drv_Motor.setPower(Math.abs(speed_strafe));
                robot.left_back_drv_Motor.setPower(Math.abs(speed_strafe));
                robot.right_back_drv_Motor.setPower(Math.abs(speed_strafe));

                // keep looping while we are still active, and there is time left, and both motors are running.
                // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
                // its target position, the motion will stop.  This is "safer" in the event that the robot will
                // always end the motion as soon as possible.
                // However, if you require that BOTH motors have finished their moves before the robot continues
                // onto the next step, use (isBusy() || isBusy()) in the loop test.
                while (opModeIsActive() &&
                        (runtime.seconds() < strafe_timeoutS) &&
                        (robot.left_front_drv_Motor.isBusy() && robot.right_front_drv_Motor.isBusy() && robot.right_back_drv_Motor.isBusy() && robot.left_back_drv_Motor.isBusy())) {

                    // Display it for the driver.
                    telemetry.addData("Path1", "Running to %7d :%7d", newLeftFrontTarget, newRightFrontTarget, newLeftBackTarget, newRightBackTarget);
                    telemetry.addData("Path2", "Running at %7d :%7d",
                            robot.left_front_drv_Motor.getCurrentPosition(),
                            robot.right_front_drv_Motor.getCurrentPosition());
                    telemetry.update();
                }


                // Stop all motion;
                robot.left_front_drv_Motor.setPower(0);
                robot.right_front_drv_Motor.setPower(0);
                robot.left_back_drv_Motor.setPower(0);
                robot.right_back_drv_Motor.setPower(0);

                // Turn off RUN_TO_POSITION
                robot.left_front_drv_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.right_front_drv_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.left_back_drv_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.right_back_drv_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                //  sleep(250);   // optional pause after each move
            }
        }
        public void intake (double speed, double intakeinches, double timeoutS){
            int newIntakeTarget;

            if (opModeIsActive()){
                newIntakeTarget = robot.intake_motor.getCurrentPosition() + (int)(intakeinches * COUNTS_PER_INCH);
                robot.intake_motor.setTargetPosition(newIntakeTarget);

                robot.intake_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                runtime.reset();
                robot.intake_motor.setPower(Math.abs(speed));
                while (opModeIsActive() &&
                        (runtime.seconds() < timeoutS) &&
                        (robot.intake_motor.isBusy()) ){

                    // Display it for the driver.
                    telemetry.addData("Path1", "Running to %7d :%7d", newIntakeTarget);
                    telemetry.addData("Path2", "Running at %7d :%7d",
                            robot.intake_motor.getCurrentPosition());
                    telemetry.update();
                }
                robot.intake_motor.setPower(0);

                robot.intake_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }

        public void launch (double speed, double fly_wheelinches, double rampinches, double timeoutS){
            int newFlyWheelTarget;
            int newRampTarget;

            if (opModeIsActive()){
                newFlyWheelTarget = robot.fly_wheel.getCurrentPosition() + (int)(fly_wheelinches * COUNTS_PER_INCH);
                newRampTarget = robot.ramp.getCurrentPosition() + (int)(rampinches * COUNTS_PER_INCH);

                robot.fly_wheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.ramp.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                runtime.reset();
                robot.fly_wheel.setPower(Math.abs(speed));
                robot.ramp.setPower(Math.abs(speed));

                while (opModeIsActive() &&
                        (runtime.seconds() < timeoutS) &&
                        (robot.intake_motor.isBusy()) ){

                    // Display it for the driver.
                    telemetry.addData("Path1", "Running to %7d :%7d", newFlyWheelTarget, newRampTarget);
                    telemetry.addData("Path2", "Running at %7d :%7d",
                            robot.fly_wheel.getCurrentPosition());
                    robot.ramp.getCurrentPosition();
                    telemetry.update();
                }

                robot.fly_wheel.setPower(0);
                robot.ramp.setPower(0);

                robot.fly_wheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.ramp.setMode(DcMotor.RunMode.RUN_USING_ENCODER);




            }
        }
    }
