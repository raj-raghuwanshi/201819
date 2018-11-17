package org.firstinspires.ftc.team13180;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Shivam Adeshara on 12/24/2017.
 */

public class RobotNavigator {
    private DcMotor Topl;
    private DcMotor Topr;
    private DcMotor Rearr;
    private DcMotor Rearl;

    public void init(HardwareMap hardwareMap) {
        Topl = hardwareMap.get(DcMotor.class, "leftMotor");
        Topr= hardwareMap.get(DcMotor.class, "leftMotor");
        Rearl = hardwareMap.get(DcMotor.class, "rightMotor");
        Rearr = hardwareMap.get(DcMotor.class, "rightMotor");
        //leftMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    public void setRunWithEncoderMode() {
        Topr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Topl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Rearl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Rearr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void moveForward(double power) {
        Topr.setPower(power);
        Topl.setPower(power);
        Rearl.setPower(power);
        Rearr.setPower(power);
    }

    public void moveBackward(double power) {
        Topr.setPower(-power);
        Topl.setPower(-power);
        Rearl.setPower(-power);
        Rearr.setPower(-power);
    }

    public void moveLeft (double power){
        leftMotor.setPower(power);
        rightMotor.setPower(-power);
    }

    public void moveRight (double power){
        leftMotor.setPower (-power);
        rightMotor.setPower(power);
    }

    public void stopMotor() {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    public void moveForwardTime(double power, long time) throws InterruptedException {
        moveForward(power);
        Thread.sleep(time);
        stopMotor();
    }

    public void moveBackwardTime(double power, long time) throws InterruptedException {
        moveBackward(power);
        Thread.sleep(time);
        stopMotor();
    }

    public void moveRightTime(double power, long time) throws InterruptedException {
        moveRight(power);
        Thread.sleep(time);
        stopMotor();
    }

    public void moveLeftTime(double power, long time) throws InterruptedException {
        moveLeft(power);
        Thread.sleep(time);
        stopMotor();
    }

    public void setLeftMotorTargetPosition(int position) {
        leftMotor.setTargetPosition(position);

    }

    public void setRightMotorTargetPosition(int position) {
        rightMotor.setTargetPosition(position);

    }

    public int getLeftMotorCurrentPosition() {
        return leftMotor.getCurrentPosition();
    }

    public int getRightMotorCurrentPosition() {
        return rightMotor.getCurrentPosition();
    }

    public void setRunToPosition() {
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setLeftMotorForwardDirection() {
        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);
    }

    public void setMotorDirection() {
        leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
    }

    public void  setLeftMotorPower(double power) {
        Topl.setPower(power);
    }

    public void  setRightMotorPower(double power) {
        Topr.setPower(power); }

    public boolean isRightMotorBusy() {
        return rightMotor.isBusy();
    }

    public boolean isLeftMotorBusy() {
        return leftMotor.isBusy();
    }
}

