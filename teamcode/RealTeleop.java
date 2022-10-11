package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="RealTeleop")

public class RealTeleop extends OpMode {

    DcMotor LFMotor;
    DcMotor RFMotor;
    DcMotor LBMotor;
    DcMotor RBMotor;
    DcMotorEx ArmMotor;
    DcMotor Carousel;
    Servo Intake;
    Double Speed;
    Double MovementSpeed;
    Servo Capstone;

    //public DistanceSensor blockAlarm;


    @Override
    public void init() {

        LFMotor = hardwareMap.get(DcMotor.class, "LFMotor");
        RFMotor = hardwareMap.get(DcMotor.class, "RFMotor");
        LBMotor = hardwareMap.get(DcMotor.class, "LBMotor");
        RBMotor = hardwareMap.get(DcMotor.class, "RBMotor");

        RFMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RBMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        ArmMotor = hardwareMap.get(DcMotorEx.class, "ArmMotor");
        Carousel = hardwareMap.get(DcMotor.class, "Carousel");
        Intake = hardwareMap.get(Servo.class, "Intake");
        Capstone = hardwareMap.get(Servo.class, "Capstone");
        //blockAlarm= hardwareMap.get(DistanceSensor.class, "BlockAlarm");

        Speed = 1.0;
        MovementSpeed = 1.0;
        Carousel.setTargetPosition(0);
        Capstone.setDirection(Servo.Direction.FORWARD);
        Capstone.setPosition(0.5);

        ArmMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }


    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */

    @Override
    public void init_loop(){


    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */

    @Override
    public void loop() {
       /* if (gamepad2.left_trigger == 1) {
            TurnSpeed = 0.5;
        } else if (gamepad2.left_trigger == 0) {
            TurnSpeed = 1.0;
        } */

        if (gamepad1.left_trigger == 1) {
            Speed = 0.3;
            MovementSpeed = 0.5;
        } else if (gamepad1.left_trigger == 0) {
            Speed = 1.0;
            MovementSpeed = 1.0;
        }

        double Pad2LeftStickY = gamepad2.left_stick_y;
        double LeftStickY = gamepad1.left_stick_y;
        double LeftStickX = -gamepad1.left_stick_x;
        double RightStickX = -gamepad1.right_stick_x;
        double Pad2RightStickY = gamepad2.right_stick_y;
        double Capstoneright = gamepad2.right_trigger;
        double Capstoneleft = gamepad2.left_trigger;

        RFMotor.setPower((-RightStickX / 1.5) + (LeftStickY - LeftStickX) * Speed);
        RBMotor.setPower((-RightStickX / 1.5 + (LeftStickY + LeftStickX)) * Speed);
        LFMotor.setPower((RightStickX / 1.5 + (LeftStickY + LeftStickX)) * Speed);
        LBMotor.setPower((RightStickX / 1.5 + (LeftStickY - LeftStickX)) * Speed);
        ArmMotor.setPower((-Pad2LeftStickY / 2));

       /* Capstone.setDirection(Servo.Direction.REVERSE);
        Capstone.setPosition((-Pad2RightStickY / 2));*/

        if (Capstoneleft == 1) {
            Capstone.setPosition(0);

       } if (Capstoneright == 1) {
            Capstone.setPosition(0.5);
        }

        //NEW CODE IDEA FOR TURNING AND MOVEMENT SEPARATION
        /*RFMotor.setPower((-RightStickX * Speed) + ((LeftStickY - LeftStickX) *  MovementSpeed));
        RBMotor.setPower((-RightStickX * Speed) + ((LeftStickY + LeftStickX) * MovementSpeed));
        LFMotor.setPower((RightStickX * Speed) + ((LeftStickY + LeftStickX) * MovementSpeed));
        LBMotor.setPower((RightStickX * Speed) + ((LeftStickY - LeftStickX) * MovementSpeed));
        ArmMotor.setPower((-Pad2LeftStickY / 2));*/
        if (gamepad2.left_bumper) {
            Intake.setPosition(0.15);
        }

        if(gamepad2.right_bumper) {
            Intake.setPosition(0.75);
        }

        /*if (gamepad2.right_bumper) {
            Carousel.setPower(-0.5);
            Carousel.setTargetPosition(0);
        } else Carousel.setPower(0);

        if (gamepad2.left_bumper) {
            Intake.setPower(0.5);
            Intake.setTargetPosition(0);
        } else Intake.setPower(0);*/

        /*if (gamepad2.a) {
            ArmMotor.setTargetPosition(200);
            ArmMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            ArmMotor.setVelocity(600);
        }

        if (gamepad2.x) {
            ArmMotor.setTargetPosition(450);
            ArmMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            ArmMotor.setVelocity(500);
        }

        if (gamepad2.y) {
            ArmMotor.setTargetPosition(625);
            ArmMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            ArmMotor.setVelocity(500);
        }*/


        if (gamepad1.right_bumper) {
            Carousel.setPower(-0.7);
            Carousel.setTargetPosition(0);
        } else Carousel.setPower(0);

        if (gamepad1.left_bumper) {
            Carousel.setPower(0.5);
            Carousel.setTargetPosition(0);
        } else Carousel.setPower(0);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop () {

    }
}