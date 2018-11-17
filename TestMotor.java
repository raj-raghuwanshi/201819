package org.firstinspires.ftc.team13180;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp
public class TestMotor extends LinearOpMode {
    private DcMotor Topl;
    private Servo TestServo;
    @Override
    public void runOpMode() {
        //imu = hardwareMap.get(Gyroscope.class, "imu");
        Topl = hardwareMap.get(DcMotor.class, "Topl");
        TestServo = hardwareMap.get(Servo.class, "TestServo");
        waitForStart();
        double tgtPower1 = 0;
        double multiPlier1 = 1;
        while (opModeIsActive()) {

            if (gamepad1.left_stick_y != 0) {
                tgtPower1 = multiPlier1 * this.gamepad1.left_stick_y;
                Topl.setPower(tgtPower1);

                if(gamepad1.y) {
                    // move to 0 degrees.
                    TestServo.setPosition(0);

                    //servoTest2.setPosition(1);
                } else if (gamepad1.a) {
                    // move to 180 degrees.
                    TestServo.setPosition(0.75);

                    //servoTest1.setPosition(1);
                    //servoTest2.setPosition(0);
                }
            }

        }

            }

        }

