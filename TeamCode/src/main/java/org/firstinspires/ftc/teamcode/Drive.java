
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Drive", group=" ")
public class Drive extends LinearOpMode{

    /**
     * class must extend "LinearOpMode" in order to apply telemetry
     * also to apply other classes such as "gamepad" and hardwaremap


     **/

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;
    private DcMotor intake;
    public void runOpMode() throws InterruptedException{
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        //oriented facing the back of the bot
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftBack  = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");

        intake = hardwareMap.get(DcMotor.class, "intake");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            // Set the motors to run in the same direction
//            leftFront.setDirection(DcMotor.Direction.FORWARD);
//            leftBack.setDirection(DcMotor.Direction.FORWARD);
//            rightFront.setDirection(DcMotor.Direction.FORWARD);
//            rightBack.setDirection(DcMotor.Direction.FORWARD);

            // Loop until the user presses "stop"
            while (opModeIsActive()) {
                // Get the joystick values
                float leftStickY = -gamepad1.left_stick_y;
                float leftStickX = gamepad1.left_stick_x;
                float rightStickX = gamepad1.right_stick_x;
                boolean rightBumper = gamepad1.right_bumper;
                // Calculate the motor powers
                float frontLeftPower = (leftStickX + leftStickY + rightStickX) / (rightBumper ? 3 : 1);
                float backLeftPower = (leftStickX - leftStickY - rightStickX) / (rightBumper ? 3 : 1);
                float frontRightPower = (leftStickX - leftStickY + rightStickX) / (rightBumper ? 3 : 1);
                float backRightPower = (leftStickX + leftStickY - rightStickX) / (rightBumper ? 3 : 1);

                // Cap the motor powers
                frontLeftPower = Range.clip(frontLeftPower, -1, 1);
                backLeftPower = Range.clip(backLeftPower, -1, 1);
                frontRightPower = Range.clip(frontRightPower, -1, 1);
                backRightPower = Range.clip(backRightPower, -1, 1);

                if(gamepad2.left_bumper){
                    intake.setPower(1);
                } else {
                    intake.setPower(0);
                }

                // Set the motor powers
                leftFront.setPower(frontLeftPower);
                leftBack.setPower(backLeftPower);
                rightFront.setPower(frontRightPower);
                rightBack.setPower(backRightPower);

                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.update();
            }
        }


    }
}
