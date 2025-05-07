public class Car {

    private double speed = 0;

    public void accelerate(double pressure) {
        speed = pressure;
    }
    
    public double getSpeed() {

        return speed;
    }
}
