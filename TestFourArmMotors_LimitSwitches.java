/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="Test FourArmMotors_LimitSwitches", group="Test")
//@Disabled
public class TestFourArmMotors_LimitSwitches extends LinearOpMode {

    /* Declare OpMode members. */
    DcMotor leftRotater;
    DcMotor rightRotater;
    DcMotor slideLeft;
    DcMotor slideRight;
    TouchSensor touchSensorRight;
    TouchSensor touchSensorLeft;
    TouchSensor touchSensorRotator;
   

    

    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        // ************* Slide MOTORS ****************
        leftRotater = hardwareMap.get(DcMotorEx.class, "leftRotater");
        rightRotater = hardwareMap.get(DcMotorEx.class, "rightRotater");
        slideLeft = hardwareMap.get(DcMotorEx.class, "slideLeft");
        slideRight = hardwareMap.get(DcMotorEx.class, "slideRight");

        touchSensorRight = hardwareMap.get(TouchSensor.class, "sensor_touchRight");
        touchSensorLeft = hardwareMap.get(TouchSensor.class, "sensor_touchLeft");
        touchSensorRotator = hardwareMap.get(TouchSensor.class, "sensor_touchRotate");


        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftRotater.setDirection(DcMotor.Direction.REVERSE);
        rightRotater.setDirection(DcMotorSimple.Direction.FORWARD);
        slideLeft.setDirection(DcMotor.Direction.REVERSE);
        slideRight.setDirection(DcMotorSimple.Direction.FORWARD);

       
        //
        leftRotater.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRotater.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRotater.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRotater.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Wait for the game to start (driver presses START)
        waitForStart();
      
        while (opModeIsActive()) {

            // Horizontal extension motors
            if (gamepad2.left_stick_y > 0.1) {
                slideLeft.setPower(gamepad2.left_stick_y);
                slideRight.setPower(gamepad2.left_stick_y);
                slideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                slideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            } else if (gamepad2.left_stick_y <= 0.1) {
                slideLeft.setPower(gamepad2.left_stick_y);
                slideRight.setPower(gamepad2.left_stick_y);
                slideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                slideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            }else
            {
                  slideLeft.setPower(0);
                  slideRight.setPower(0);
            }


            //Rotater Motors
            if (gamepad2.right_stick_y> 0.1) {
                rightRotater.setPower(gamepad2.right_stick_y);
                leftRotater.setPower(gamepad2.right_stick_y);
                rightRotater.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                leftRotater.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            } else if (gamepad2.right_stick_y <= -0.1) {
                rightRotater.setPower(gamepad2.right_stick_y);
                leftRotater.setPower(gamepad2.right_stick_y);
                rightRotater.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                leftRotater.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            }else
            {
                rightRotater.setPower(0);
                leftRotater.setPower(0);
            }
            
            // TOUCH SENSORS
            if (touchSensorRight.isPressed()) {
                telemetry.addData("Touch Sensor Right", "Is Pressed");
                slideRight.setPower(0);
                slideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            
            } else {
                slideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                telemetry.addData("Touch Sensor Right", "Is Not Pressed");
            }
            if (touchSensorLeft.isPressed()) {
                telemetry.addData("Touch Sensor Left", "Is Pressed");
                slideLeft.setPower(0);
                slideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            } else {
                slideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                telemetry.addData("Touch Sensor Left", "Is Not Pressed");
            }

            //Rotaters Touch Sensor. Only Use ONE because of the connection.
            if (touchSensorRotator.isPressed()) {
                telemetry.addData("Touch Sensor Rotate", "Is Pressed");
                leftRotater.setPower(0);
                rightRotater.setPower(0);
                leftRotater.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightRotater.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            } else {
                leftRotater.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightRotater.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                telemetry.addData("Touch Sensor Rotater", "Is Not Pressed");
            }



            telemetry.addData("Motor Ticks slideLeft: ", slideLeft.getCurrentPosition());
            telemetry.addData("Motor Ticks slideRight: ", slideRight.getCurrentPosition());
            telemetry.addData("Motor Ticks rightRotater: ", rightRotater.getCurrentPosition());
            telemetry.addData("Motor Ticks leftRotater: ", leftRotater.getCurrentPosition());

            telemetry.addData(">", "Rotater Motors use right_stick_y");
            telemetry.addData(">", "Horizontal Extension Motors use left_stick_y");
           


            telemetry.update();


        }
    }
}
