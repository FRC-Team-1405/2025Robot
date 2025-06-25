public class Car {
    
    private double speed = 0;
    public void accelerate(double pressure) {
        speed = Math.round(pressure * 0.78);
    }

    

    public double getSpeed() {
        return speed;
    }
}
