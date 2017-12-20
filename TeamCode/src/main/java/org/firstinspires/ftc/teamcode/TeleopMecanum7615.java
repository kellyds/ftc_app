/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;




@TeleOp(name="TeleopMecanum7615", group="Iterative Opmode")
//@Disabled
public class TeleopMecanum7615 extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    //private DcMotor leftDrive = null;
    //private DcMotor rightDrive = null;

    DcMotor LFront; //Forward left
    DcMotor LRear; //Rear Left
    DcMotor RFront; //Forward Right
    DcMotor RRear; //Rear Right
    DcMotor Arm; // Arm Up/Down
    DcMotor RIntake;
    DcMotor LIntake;
    Servo jewel;
    ColorSensor sensorColor;
    DistanceSensor sensorDistance;
    //DcMotor Lift;

    // Servo Claw;
    //For raising color sensor after Auto

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        LFront = hardwareMap.dcMotor.get ("LFront");
        LRear = hardwareMap.dcMotor.get ("LRear");
        RFront = hardwareMap.dcMotor.get ("RFront");
        RRear = hardwareMap.dcMotor.get ("RRear");
        Arm = hardwareMap.dcMotor.get("Arm");
        LFront.setDirection(DcMotor.Direction.REVERSE);
        LRear.setDirection(DcMotor.Direction.REVERSE);
        // Claw = hardwareMap.servo.get("Claw");
        Arm.setDirection(DcMotor.Direction.FORWARD);
        RIntake= hardwareMap.dcMotor.get ("RIntake");
        LIntake = hardwareMap.dcMotor.get ("LIntake");
        //Lift = hardwareMap.dcMotor.get("Lift");
        jewel = hardwareMap.servo.get("jewel");
        sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");

        // get a reference to the distance sensor that shares the same name.
        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");
        sensorColor.enableLed(true);

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
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        telemetry.addData("Status", "Running: " + runtime.toString());

        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = gamepad1.right_stick_x;
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        LFront.setPower(v1);
        RFront.setPower(v2);
        LRear.setPower(v3);
        RRear.setPower(v4);

        if (gamepad1.x) {
            jewel.setPosition(0);
        }
        if (gamepad1.b) {
            jewel.setPosition(1);

            if (gamepad1.y) {
                Arm.setPower(.5);
            }

            if (gamepad1.a) {
                Arm.setPower(-.5);
            } else {
                Arm.setPower(0);
            }
            //
            //intake code

            if (gamepad1.right_bumper) {
                RIntake.setPower(1);
                LIntake.setPower((-1));
            } else if (gamepad1.left_bumper
                    ) {
                RIntake.setPower(-1);
                LIntake.setPower((1));
            } else {
                RIntake.setPower(0);
                LIntake.setPower(0);
            }

        }
    }


    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop(){
    }
}