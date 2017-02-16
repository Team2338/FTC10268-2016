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
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Template: Iterative OpMode", group="Iterative Opmode")  // @Autonomous(...) is the other common choice
@Disabled
public class TeleOp extends OpMode
{
    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;
    DcMotor catapult1;
    DcMotor catapult2;
    DcMotor collector;
    Servo elevator;
    Servo scoopie;
    Servo tail;
//    ColorSensor color;

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    // private DcMotor leftMotor = null;
    // private DcMotor rightMotor = null;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        catapult1 = hardwareMap.dcMotor.get("shooter1");
        catapult2 = hardwareMap.dcMotor.get("shooter2");
        collector = hardwareMap.dcMotor.get("staging");
        elevator = hardwareMap.servo.get("collector");
        scoopie = hardwareMap.servo.get("scoopee");
        tail = hardwareMap.servo.get("tailMotor");
//        color = hardwareMap.colorSensor.get("Color");
//        color.enableLed(true);

        telemetry.addData("Status", "Initialized");

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

        runtime.reset();
        telemetry.addLine("TeleOp Started");
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        telemetry.addData("Status", "Running: " + runtime.toString());

        // Driver Controls
        /*
            Literally just driving
         */
        frontLeftMotor.setPower(-gamepad1.left_stick_y);
        backLeftMotor.setPower(-gamepad1.left_stick_y);
        frontRightMotor.setPower(-gamepad1.right_stick_y);
        backRightMotor.setPower(-gamepad1.right_stick_y);

        // Aux Controls
        /*
            Right Bumper = Catapult
            Left Bumper = Scoopie
            A = Collect
            B = Eject
            Start = Tail Down
            Back = Tail Up
           */

        if(gamepad2.right_bumper){
            catapult1.setPower(1);
            catapult2.setPower(1);
        } else {
            catapult1.setPower(0);
            catapult2.setPower(0);
        }

        if(gamepad2.a){
            collector.setPower(1);
            elevator.setDirection(Servo.Direction.FORWARD);
            elevator.setPosition(0);
        } else if(gamepad2.b) {
            collector.setPower(-1);
            elevator.setDirection(Servo.Direction.REVERSE);
            elevator.setPosition(0);
        } else {
            collector.setPower(0);
            elevator.setPosition(.5);
        }

        if(gamepad2.left_bumper){
            scoopie.setPosition(90);
        } else {
            scoopie.setPosition(0);
        }

        if(gamepad2.start){
            tail.setDirection(Servo.Direction.FORWARD);
            tail.setPosition(0);
        } else if(gamepad2.back){
            tail.setDirection(Servo.Direction.REVERSE);
            tail.setPosition(0);
        } else {
            tail.setPosition(.5);
        }

//        telemetry.addData("Clear", color.alpha());
//        telemetry.addData("Red  ", color.red());
//        telemetry.addData("Green", color.green());
//        telemetry.addData("Blue ", color.blue());

        telemetry.addData("Right Position", frontRightMotor.getCurrentPosition());
        telemetry.addData("Left Position", frontLeftMotor.getCurrentPosition());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
