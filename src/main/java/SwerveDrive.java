

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Swerve Drive subsystem 
 * Need to create a custom PID source
 * Carry on from 'Writing the DriveContinuous Command'
 */
public class SwerveDrive extends SubsystemBase
{
    // Motors
    private static CANTalon LEFT_FRONT_DRIVE_SPEED_MOTOR;
    private static CANTalon LEFT_BACK_DRIVE_SPEED_MOTOR;
    private static CANTalon RIGHT_FRONT_DRIVE_SPEED_MOTOR;
    private static CANTalon RIGHT_BACK_DRIVE_SPEED_MOTOR;

    private static CANTalon LEFT_FRONT_DRIVE_DIRECTION_MOTOR;
    private static CANTalon LEFT_BACK_DRIVE_DIRECTION_MOTOR;
    private static CANTalon RIGHT_FRONT_DRIVE_DIRECTION_MOTOR;
    private static CANTalon RIGHT_BACK_DRIVE_DIRECTION_MOTOR;

    // Encoders
    public static Encoder LEFT_FRONT_DRIVE_DISTANCE_ENCODER;
    public static Encoder LEFT_BACK_DRIVE_DISTANCE_ENCODER;
    public static Encoder RIGHT_FRONT_DRIVE_DISTANCE_ENCODER;
    public static Encoder RIGHT_BACK_DRIVE_DISTANCE_ENCODER;
    public static MedianPIDSource DRIVE_DISTANCE_ENCODERS;

    public static Encoder LEFT_FRONT_DRIVE_DIRECTION_ENCODER;
    public static Encoder LEFT_BACK_DRIVE_DIRECTION_ENCODER;
    public static Encoder RIGHT_FRONT_DRIVE_DIRECTION_ENCODER;
    public static Encoder RIGHT_BACK_DRIVE_DIRECTION_ENCODER;

    // Direction encoder wrapper that scales to degrees
    public static PIDSourceExtended LEFT_FRONT_DRIVE_DIRECTION_SCALED;
    public static PIDSourceExtended LEFT_BACK_DRIVE_DIRECTION_SCALED;
    public static PIDSourceExtended RIGHT_FRONT_DRIVE_DIRECTION_SCALED;
    public static PIDSourceExtended RIGHT_BACK_DRIVE_DIRECTION_SCALED;

    // Gyro
    public static AHRS DRIVE_GYRO;

    SwerveDriveWheel LEFT_FRONT_DRIVE_WHEEL;
    SwerveDriveWheel LEFT_BACK_DRIVE_WHEEL;
    SwerveDriveWheel RIGHT_FRONT_DRIVE_WHEEL;
    SwerveDriveWheel RIGHT_BACK_DRIVE_WHEEL;
    public SwerveDriveCoordinator SWERVE_DRIVE_COORDINATOR;

    public SwerveDrive()
    {
        // Motors
        LEFT_FRONT_DRIVE_SPEED_MOTOR = new CANTalon(RobotMap.LEFT_FRONT_DRIVE_SPEED_MOTOR_PIN);
        LEFT_BACK_DRIVE_SPEED_MOTOR = new CANTalon(RobotMap.LEFT_BACK_DRIVE_SPEED_MOTOR_PIN);
        RIGHT_FRONT_DRIVE_SPEED_MOTOR = new CANTalon(RobotMap.RIGHT_FRONT_DRIVE_SPEED_MOTOR_PIN);
        RIGHT_BACK_DRIVE_SPEED_MOTOR = new CANTalon(RobotMap.RIGHT_BACK_DRIVE_SPEED_MOTOR_PIN);

        LEFT_FRONT_DRIVE_DIRECTION_MOTOR = new CANTalon(RobotMap.LEFT_FRONT_DRIVE_DIRECTION_MOTOR_PIN);
        LEFT_BACK_DRIVE_DIRECTION_MOTOR = new CANTalon(RobotMap.LEFT_BACK_DRIVE_DIRECTION_MOTOR_PIN);
        RIGHT_FRONT_DRIVE_DIRECTION_MOTOR = new CANTalon(RobotMap.RIGHT_FRONT_DRIVE_DIRECTION_MOTOR_PIN);
        RIGHT_BACK_DRIVE_DIRECTION_MOTOR = new CANTalon(RobotMap.RIGHT_BACK_DRIVE_DIRECTION_MOTOR_PIN);

        // Encoders
        LEFT_FRONT_DRIVE_DISTANCE_ENCODER = new Encoder(RobotMap.LEFT_FRONT_DRIVE_DISTANCE_ENCODER_PIN_A, RobotMap.LEFT_FRONT_DRIVE_DISTANCE_ENCODER_PIN_B);
        LEFT_BACK_DRIVE_DISTANCE_ENCODER = new Encoder(RobotMap.LEFT_BACK_DRIVE_DISTANCE_ENCODER_PIN_A, RobotMap.LEFT_BACK_DRIVE_DISTANCE_ENCODER_PIN_B);
        RIGHT_FRONT_DRIVE_DISTANCE_ENCODER = new Encoder(RobotMap.RIGHT_FRONT_DRIVE_DISTANCE_ENCODER_PIN_A, RobotMap.RIGHT_FRONT_DRIVE_DISTANCE_ENCODER_PIN_B);
        RIGHT_BACK_DRIVE_DISTANCE_ENCODER = new Encoder(RobotMap.RIGHT_BACK_DRIVE_DISTANCE_ENCODER_PIN_A, RobotMap.RIGHT_BACK_DRIVE_DISTANCE_ENCODER_PIN_B);
        DRIVE_ENCODERS = new MedianPIDSource(LEFT_FRONT_DRIVE_DISTANCE_ENCODER, LEFT_BACK_DRIVE_DISTANCE_ENCODER, RIGHT_FRONT_DRIVE_DISTANCE_ENCODER, RIGHT_BACK_DRIVE_DISTANCE_ENCODER);

        LEFT_FRONT_DRIVE_DIRECTION_ENCODER = new Encoder(RobotMap.LEFT_FRONT_DRIVE_DIRECTION_ENCODER_PIN_A, RobotMap.LEFT_FRONT_DRIVE_DIRECTION_ENCODER_PIN_B);
        LEFT_BACK_DRIVE_DIRECTION_ENCODER = new Encoder(RobotMap.LEFT_BACK_DRIVE_DIRECTION_ENCODER_PIN_A, RobotMap.LEFT_BACK_DRIVE_DIRECTION_ENCODER_PIN_B);
        RIGHT_FRONT_DRIVE_DIRECTION_ENCODER = new Encoder(RobotMap.RIGHT_FRONT_DRIVE_DIRECTION_ENCODER_PIN_A, RobotMap.RIGHT_FRONT_DRIVE_DIRECTION_ENCODER_PIN_B);
        RIGHT_BACK_DRIVE_DIRECTION_ENCODER = new Encoder(RobotMap.RIGHT_BACK_DRIVE_DIRECTION_ENCODER_PIN_A, RobotMap.RIGHT_BACK_DRIVE_DIRECTION_ENCODER_PIN_B);

        // Direction encoder wrapper that scales to degrees
        LEFT_FRONT_DRIVE_DIRECTION_SCALED = new PIDSourceExtended(LEFT_FRONT_DRIVE_DIRECTION_ENCODER);
        LEFT_BACK_DRIVE_DIRECTION_SCALED = new PIDSourceExtended(LEFT_BACK_DRIVE_DIRECTION_ENCODER);
        RIGHT_FRONT_DRIVE_DIRECTION_SCALED = new PIDSourceExtended(RIGHT_FRONT_DRIVE_DIRECTION_ENCODER);
        RIGHT_BACK_DRIVE_DIRECTION_SCALED = new PIDSourceExtended(RIGHT_BACK_DRIVE_DIRECTION_ENCODER);

        // Gyro
        DRIVE_GYRO = new AHRS(RobotMap.MXP_PORT);

        // SwerveDriveWheels
        double wheelP = 0.02;
        double wheelI = 0.001;
        double wheelD = 0.0;
        LEFT_FRONT_DRIVE_WHEEL = new SwerveDriveWheel(wheelP, wheelI, wheelD, LEFT_FRONT_DRIVE_DIRECTION_SCALED, LEFT_FRONT_DRIVE_DIRECTION_MOTOR, LEFT_FRONT_DRIVE_SPEED_MOTOR);
        LEFT_BACK_DRIVE_WHEEL = new SwerveDriveWheel(wheelP, wheelI, wheelD, LEFT_BACK_DRIVE_DIRECTION_SCALED, LEFT_BACK_DRIVE_DIRECTION_MOTOR, LEFT_BACK_DRIVE_SPEED_MOTOR);
        RIGHT_FRONT_DRIVE_WHEEL = new SwerveDriveWheel(wheelP, wheelI, wheelD, RIGHT_FRONT_DRIVE_DIRECTION_SCALED, RIGHT_FRONT_DRIVE_DIRECTION_MOTOR, RIGHT_FRONT_DRIVE_SPEED_MOTOR);
        RIGHT_BACK_DRIVE_WHEEL = new SwerveDriveWheel(wheelP, wheelI, wheelD, RIGHT_BACK_DRIVE_DIRECTION_SCALED, RIGHT_BACK_DRIVE_DIRECTION_MOTOR, RIGHT_BACK_DRIVE_SPEED_MOTOR);
        // SwerveDriveCoordinator
        SWERVE_DRIVE_COORDINATOR = new SwerveDriveCoordinator(LEFT_FRONT_DRIVE_WHEEL, LEFT_BACK_DRIVE_WHEEL, RIGHT_FRONT_DRIVE_WHEEL, RIGHT_BACK_DRIVE_WHEEL);
    }
    
    // public void initDefaultCommand()
    // {}

}


/**
 * Swerve Drive Wheel
 * Custom class that represents a swerve drive wheel
*/
public class SwerveDrive extends SwerveDriveWheel
{
    public PIDController directionController;
    public PIDOutput directionMotor;
    public PIDSource directionSensor;

    // PID Controller - takes input from the wheel direction encoder & outputs to the wheel direction motor

    public SwerveDriveWheel(double P, double I, double D, PIDSource directionSensor, PIDOutput directionMotor)
    {
        this.directionSensor = directionSensor;
        this.directionMotor = directionMotor;
        directionController = new PIDController(P, I, D, directionSensor, directionMotor);
    }
    
    public void setDirection(double setpoint)
    {
        directionController.reset();
        double currentAngle = directionSensor.get();
        // finds closest angle to setpoint
        double setpointAngle = closestAngle(currentAngle, setpoint);
        // finds closest angle to setpoint + 180
        double setpointAngleFlipped = closestAngle(currentAngle, setpoint + 180.0);
        // if the closest angle to setpoint is shorter
        if (Math.abs(setpointAngle) <= Math.abs(setpointAngleFlipped))
        {
            // unflip the motor direction use the setpoint
            directionMotor.setGain(1.0);
            directionController.setSetpoint(currentAngle + setpointAngle);
        }
        // if the closest angle to setpoint + 180 is shorter
        else
        {
            // flip the motor direction and use the setpoint + 180
            directionMotor.setGain(-1.0);
            directionController.setSetpoint(currentAngle + setpointAngleFlipped);
        }
        directionController.enable();
    }

    public void setSpeed(double speed)
    {
        speedMotor.set(speed);
    }
}


/**
 * Calculates the shortest/fastest way to get between two angles
 */
private static double closestAngle(double a, double b)
{
        // get direction
        double dir = modulo(b, 360.0) - modulo(a, 360.0);

        // convert from -360 to 360 to -180 to 180
        if (Math.abs(dir) > 180.0)
        {
                dir = -(Math.signum(dir) * 360.0) + dir;
        }
        return dir;
}

/**
 * Calculates the shortest/fastest way to get between two angles
 */
public void setDirection(double setpoint)
{
    directionController.reset();

    // Uses most efficient way to get from current wheel angle to the desired wheel angle
    double currentAngle = directionSensor.get();
    directionController.setSetpoint(currentAngle + closestAngle(currentAngle, setpoint));

    directionController.enable();
}

public class SwerveDrive extends SwerveDriveCoordinator
{
    SwerveDriveWheel leftFrontWheel;
    SwerveDriveWheel leftBackWheel;
    SwerveDriveWheel rightFrontWheel;
    SwerveDriveWheel rightBackWheel;

    public SwerveDriveCoordinator(SwerveDriveWheel leftFrontWheel, SwerveDriveWheel leftBackWheel, SwerveDriveWheel rightFrontWheel, SwerveDriveWheel rightBackWheel)
    {
        this.leftFrontWheel = leftFrontWheel;
        this.leftBackWheel = leftBackWheel;
        this.rightFrontWheel = rightFrontWheel;
        this.rightBackWheel = rightBackWheel;
    }

    public void translate(double direction, double power)
    // pointing the wheels in the desired direction
    {
    leftFrontWheel.setDirection(direction);
    leftBackWheel.setDirection(direction);
    rightFrontWheel.setDirection(direction);
    rightBackWheel.setDirection(direction);

    leftFrontWheel.setSpeed(power);
    leftBackWheel.setSpeed(power);
    rightFrontWheel.setSpeed(power);
    rightBackWheel.setSpeed(power);

    public void inplaceTurn(double power)
    // for turning in place
    {
    leftFrontWheel.setDirection(135.0);
    leftBackWheel.setDirection(45.0);
    rightFrontWheel.setDirection(-45.0);
    rightBackWheel.setDirection(-135.0);

    leftFrontWheel.setSpeed(power);
    leftBackWheel.setSpeed(power);
    rightFrontWheel.setSpeed(power);
    rightBackWheel.setSpeed(power);
    }
    }

    public void translateTurn(double direction, double translatePower, double turnPower)
    {
    double turnAngle = turnPower * 45.0;

    // if the left front wheel is in the front
    if (closestAngle(direction, 135.0) >= 90.0)
    {
        leftFrontWheel.setDirection(direction + turnAngle);
    }
    // if it's in the back
    else
    {
        leftFrontWheel.setDirection(direction - turnAngle);
    }
    // if the left back wheel is in the front
    if (closestAngle(direction, 225.0) > 90.0)
    {
        leftBackWheel.setDirection(direction + turnAngle);
    }
    // if it's in the back
    else
    {
        leftBackWheel.setDirection(direction - turnAngle);
    }
    // if the right front wheel is in the front
    if (closestAngle(direction, 45.0) > 90.0)
    {
        rightFrontWheel.setDirection(direction + turnAngle);
    }
    // if it's in the back
    else
    {
        rightFrontWheel.setDirection(direction - turnAngle);
    }
    // if the right back wheel is in the front
    if (closestAngle(direction, 315.0) >= 90.0)
    {
        rightBackWheel.setDirection(direction + turnAngle);
    }
    // if it's in the back
    else
    {
        rightBackWheel.setDirection(direction - turnAngle);
    }

    leftFrontWheel.setSpeed(translatePower);
    leftBackWheel.setSpeed(translatePower);
    rightFrontWheel.setSpeed(translatePower);
    rightBackWheel.setSpeed(translatePower);
    }

    public void setSwerveDrive(double direction, double translatePower, double turnPower)
    {
    if ((translatePower == 0.0) && (turnPower != 0.0))
    {
        inplaceTurn(turnPower);
    }
    else
    {
        translateTurn(direction, translatePower, turnPower);
    }
    }
}