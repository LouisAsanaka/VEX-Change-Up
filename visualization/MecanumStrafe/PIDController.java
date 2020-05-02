public class PIDController {
    
    public static class Gains {
        float kP, kI, kD;
        
        public Gains(float kP, float kI, float kD) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
        }
    }
    
    private float kP, kI, kD;
    private float setpoint;
    
    private float posTol = 3.0f;
    private float velTol = Float.POSITIVE_INFINITY;
    
    private float previousError, totalError, positionError, velocityError;
    
    public float clamp(float val, float min, float max) {
        return Math.max(min, Math.min(max, val));
    }

    public PIDController(Gains gains) {
        kP = gains.kP;
        kI = gains.kI;
        kD = gains.kD;
    }
    
    public void setSetpoint(float setpoint) {
        this.setpoint = setpoint;
    }
    
    public boolean atSetpoint() {
        return Math.abs(positionError) < posTol && Math.abs(velocityError) < velTol; 
    }
    
    public float calculate(float measurement, float dt) {
        previousError = positionError;

        positionError = setpoint - measurement;
        velocityError = (positionError - previousError) / dt;
    
        totalError += positionError;
        totalError = clamp(totalError, -1.0f, 1.0f);
    
        return clamp(kP * positionError + kI * totalError + kD * velocityError, -1.0f, 1.0f);
    }
    
    public void reset() {
        previousError = 0.0f;
        positionError = 0.0f;
        velocityError = 0.0f;
        totalError = 0.0f;
    }
}
